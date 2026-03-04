#!/usr/bin/env python3
import sys
import time
import os
import math
import subprocess
import rclpy

import btr2_refbox
import kachaka_api

from geometry_msgs.msg import Pose2D
from refbox_msgs.msg import MachineReportEntryBTR
from refbox_msgs.srv import SendPrepareMachine

TEAMNAME = "BabyTigers-R"

# Challenge Track field
FIELDMINX = -5
FIELDMAXX = 5
FIELDMINY = 1
FIELDMAXY = 5

MAXSTEP = 999
FalseValue = 9999


class btr2_rcll(object):
    def __init__(self, teamName="Babytigers-R", robotNum=1, gazeboFlag=False, refbox=None):
        self.teamName = teamName
        self.robotNum = int(robotNum)
        self.gazeboFlag = bool(gazeboFlag)

        if refbox is None:
            raise RuntimeError("refbox must be provided")
        self.refbox = refbox

        self.machineReport = MachineReportEntryBTR()
        self.prepareMachine = SendPrepareMachine.Request()

        # Kachaka
        self.kachakaIP = os.getenv("kachaka_IP")
        if not self.kachakaIP:
            raise RuntimeError("Environment variable kachaka_IP is not set")

        self.kachaka = kachaka_api.KachakaApiClient(target=self.kachakaIP + ":26400")
        self.kachaka.set_auto_homing_enabled(False)
        self.kachaka.get_battery_info()

        self.refbox.get_logger().info(f"[init] kachaka_IP={self.kachakaIP}")

    # --------------------------
    # RefBox helper
    # --------------------------
    def spin_and_beacon(self, timeout_sec: float = 0.1):
        """Keep refbox callbacks flowing and request updates. Safe even if service not ready."""
        try:
            rclpy.spin_once(self.refbox, timeout_sec=timeout_sec)
        except Exception:
            pass
        try:
            self.refbox.sendBeacon()
        except Exception:
            # service /rcll/send_beacon not available yet, etc.
            pass

    def wait_until(self, cond_fn, timeout: float, msg: str):
        t0 = time.time()
        while not cond_fn():
            self.spin_and_beacon()
            if time.time() - t0 > timeout:
                self.refbox.get_logger().warn(f"[timeout] {msg}")
                return False
        return True

    # --------------------------
    # Zone conversion (Humble-safe)
    # --------------------------
    def zoneToXY(self, zone: int) -> Pose2D:
        """
        zone: e.g. 13 or 1013
        returns Pose2D(x,y) where x,y are float
        """
        z = abs(int(zone))
        y = float(z % 10)
        x = float((z % 100) // 10)

        # 1000+ is negative-x side in your convention
        if int(zone) > 1000:
            x = -x

        p = Pose2D()
        p.x = float(x)
        p.y = float(y)
        p.theta = 0.0
        return p

    # --------------------------
    # Kachaka coordinate transforms
    # --------------------------
    def field2kachaka(self, pose: Pose2D) -> Pose2D:
        k = Pose2D()
        if self.refbox.teamColorName == "C":
            k.x = float(pose.y - 0.5)
            k.y = float(-pose.x + 2.5)
        else:
            k.x = float(pose.y - 0.5)
            k.y = float(-pose.x + 2.5 - 5.0)
        k.theta = float(pose.theta - math.pi / 2.0)
        return k

    def kachaka2field(self, pose: Pose2D) -> Pose2D:
        f = Pose2D()
        if self.refbox.teamColorName == "C":
            f.x = float(-pose.y + 2.5)
            f.y = float(pose.x + 0.5)
        else:
            f.x = float(-pose.y + 2.5 - 5.0)
            f.y = float(pose.x + 0.5)
        f.theta = float(pose.theta + math.pi / 2.0)
        return f

    def kachaka_get_robot_pose(self, coordinate: str) -> Pose2D:
        self.spin_and_beacon()
        pose = self.kachaka.get_robot_pose()
        if coordinate == "kachaka":
            return pose
        if coordinate == "field":
            return self.kachaka2field(pose)
        self.refbox.get_logger().warn(f"[kachaka_get_robot_pose] unknown coordinate={coordinate}, fallback to field")
        return self.kachaka2field(pose)

    def kachaka_stop_status(self, pose_ref: Pose2D, precision: int = 100) -> bool:
        """True if current pose is (almost) same as pose_ref. round-based quantization safe for negatives."""
        self.spin_and_beacon()
        pose_now = self.kachaka_get_robot_pose("kachaka")

        def q(v: float) -> int:
            return int(round(float(v) * precision))

        return (q(pose_ref.x) == q(pose_now.x) and
                q(pose_ref.y) == q(pose_now.y) and
                q(pose_ref.theta) == q(pose_now.theta))

    def kachaka_speak(self, text: str):
        self.spin_and_beacon()
        try:
            self.kachaka.speak(text)
        except Exception as e:
            self.refbox.get_logger().warn(f"[kachaka_speak] failed: {e}")

    # --------------------------
    # theta utility
    # --------------------------
    def ensure_rad(self, theta: float) -> float:
        """
        safety: if someone accidentally passes degree (e.g. 90, 180),
        convert when abs(theta) > 2*pi and <= 360*something.
        """
        th = float(theta)
        if abs(th) > 2 * math.pi and abs(th) <= 720.0:
            return math.radians(th)
        return th

    # --------------------------
    # Move command (API-first, fallback to btr2_kachaka.py)
    # --------------------------
    def kachaka_move_to_pose(self, x: float, y: float, theta: float,
                             start_timeout: float = 5.0, done_timeout: float = 120.0) -> bool:
        """
        Prefer direct RPC (self.kachaka.move_to_pose). If not available, fallback to btr2_kachaka.py.
        """
        self.spin_and_beacon()
        theta = self.ensure_rad(theta)

        self.refbox.get_logger().info(f"[kachaka_move_to_pose field] ({x}, {y}, {theta})")

        target_field = Pose2D()
        target_field.x = float(x)
        target_field.y = float(y)
        target_field.theta = float(theta)

        target_k = self.field2kachaka(target_field)
        self.refbox.get_logger().info(f"[kachaka_move_to_pose kachaka] ({target_k.x}, {target_k.y}, {target_k.theta})")

        # already arrived?
        if self.kachaka_stop_status(target_k, precision=100):
            self.refbox.get_logger().info("[kachaka_move_to_pose] already arrived")
            return True

        issued = False
        # 1) try direct API
        try:
            if hasattr(self.kachaka, "move_to_pose"):
                self.kachaka.move_to_pose(target_k.x, target_k.y, target_k.theta)
                issued = True
            elif hasattr(self.kachaka, "move_to_position"):
                # some clients use different name
                self.kachaka.move_to_position(target_k.x, target_k.y, target_k.theta)
                issued = True
        except Exception as e:
            self.refbox.get_logger().warn(f"[kachaka_move_to_pose] direct API failed: {e}")

        # 2) fallback: run btr2_kachaka.py (this is what you confirmed works)
        if not issued:
            cmd = ["python3", "btr2_kachaka.py", "move_to_pose",
                   str(target_k.x), str(target_k.y), str(target_k.theta)]
            self.refbox.get_logger().info(f"[kachaka_move_to_pose] fallback cmd={' '.join(cmd)}")
            try:
                # do NOT redirect to /dev/null, keep error visible if it fails
                subprocess.run(cmd, check=True)
                issued = True
            except Exception as e:
                self.refbox.get_logger().error(f"[kachaka_move_to_pose] fallback failed: {e}")
                return False

        # wait start
        t0 = time.time()
        while not self.kachaka.is_command_running():
            self.spin_and_beacon()
            if time.time() - t0 > start_timeout:
                self.refbox.get_logger().warn("[kachaka_move_to_pose] did not start (timeout)")
                break

        # wait finish
        t1 = time.time()
        while self.kachaka.is_command_running():
            self.spin_and_beacon()
            if time.time() - t1 > done_timeout:
                self.refbox.get_logger().warn("[kachaka_move_to_pose] running too long (timeout)")
                break

        ok = self.kachaka_stop_status(target_k, precision=50)
        self.refbox.get_logger().info(f"[kachaka_move_to_pose] finished ok={ok}")
        return ok

    # --------------------------
    # Navigation helpers
    # --------------------------
    def navToPoint(self, point: Pose2D) -> bool:
        """
        grid cell center -> field continuous -> move
        """
        self.kachaka_speak(f"Go to Zone {point.x} {point.y}")

        px = float(point.x)
        py = float(point.y)

        # cell center convention (your original)
        if px > 0:
            px = float(int(px) - 0.5)
        else:
            px = float(int(px) + 0.5)
        py = float(int(py) - 0.5)

        theta = self.ensure_rad(float(point.theta))
        ok = self.kachaka_move_to_pose(px, py, theta)
        self.kachaka_speak("finished")
        return ok

    def get_route_snapshot(self):
        """Always read latest route (route can update while running)."""
        self.spin_and_beacon()
        route = list(self.refbox.refboxNavigationRoutes.route)
        return route

    def getNextPoint(self, idx: int) -> Pose2D:
        route = self.get_route_snapshot()
        if not route:
            p = Pose2D()
            p.x = 0.0
            p.y = 0.0
            p.theta = 0.0
            return p

        i = min(int(idx), len(route) - 1)
        zone = route[i].zone
        p = self.zoneToXY(zone)
        p.theta = 0.0  # (route theta not provided in msg; keep 0)
        self.refbox.get_logger().info(f"[getNextPoint] idx={i}/{len(route)} zone={zone} -> ({p.x},{p.y})")
        return p

    # --------------------------
    # Challenge Track: navigation only
    # --------------------------
    def navigation(self):
        self.refbox.get_logger().info("[navigation] start")

        # Wait for machine info & route (robust)
        self.wait_until(lambda: len(self.refbox.refboxMachineInfo.machines) > 0, 15.0,
                        "MachineInfo not received (check refbox connection)")
        self.wait_until(lambda: len(self.refbox.refboxNavigationRoutes.route) > 0, 15.0,
                        "NavigationRoutes not received (check refbox route publisher)")

        route0 = self.get_route_snapshot()
        if not route0:
            self.refbox.get_logger().error("[navigation] route is empty. abort.")
            return

        self.refbox.get_logger().info(f"[navigation] route_len={len(route0)} zones={[r.zone for r in route0]}")

        # Execute sequentially; refresh route each time
        i = 0
        while True:
            route = self.get_route_snapshot()
            if not route:
                self.refbox.get_logger().warn("[navigation] route became empty -> done")
                break
            if i >= len(route):
                self.refbox.get_logger().info("[navigation] reached end of route -> done")
                break

            p = self.getNextPoint(i)
            self.refbox.get_logger().info(f"[navigation] go point#{i} -> ({p.x},{p.y})")
            ok = self.navToPoint(p)

            if not ok:
                self.refbox.get_logger().warn(f"[navigation] failed point#{i}, retry once")
                if not self.navToPoint(p):
                    self.refbox.get_logger().error(f"[navigation] abort at point#{i}")
                    break

            i += 1

        self.refbox.get_logger().info("[navigation] done")

    # --------------------------
    # Challenge entry
    # --------------------------
    def challenge(self, name: str):
        funcs = {
            "navigation": self.navigation,
        }

        # start pose (challenge track)
        pose = Pose2D()
        pose.x = -1.0 * self.robotNum - 1.5
        pose.y = 0.5
        pose.theta = math.pi / 2.0  # rad

        # wait team color info a bit
        self.spin_and_beacon()
        self.refbox.get_logger().info(f"[challenge] TeamColor={getattr(self.refbox, 'teamColor', None)} name={getattr(self.refbox, 'teamColorName', None)}")

        # mirror (your original logic)
        if getattr(self.refbox, "teamColor", 0) == 1:
            pose.x = -pose.x

        self.kachaka_speak(f"{name} を開始します")
        self.kachaka_move_to_pose(pose.x, pose.y + 1.0, pose.theta)

        if name not in funcs:
            self.refbox.get_logger().error(f"[challenge] Unknown challenge: {name}")
            return
        funcs[name]()


def main(args=None):
    rclpy.init(args=args)

    argv = sys.argv
    if len(argv) < 2:
        print("Usage: python3 btr2_rcll2025_fixed.py navigation [robotNum]")
        return

    challenge_name = argv[1]
    robotNum = int(argv[2]) if len(argv) >= 3 else 1

    gazeboFlag = False  # real robot

    refbox = btr2_refbox.refbox(teamName=TEAMNAME, robotNum=robotNum, gazeboFlag=gazeboFlag)

    rcll = btr2_rcll(teamName=TEAMNAME, robotNum=robotNum, gazeboFlag=gazeboFlag, refbox=refbox)
    rcll.kachaka_speak("こんにちは。btr2_rcll2025_fixed を実行中です。")

    rcll.challenge(challenge_name)

    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()