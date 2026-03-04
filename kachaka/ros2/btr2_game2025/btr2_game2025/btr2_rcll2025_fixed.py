#!/usr/bin/env python3
import sys
import time
import os
import math
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

FIELDSIZEX = (FIELDMAXX - FIELDMINX) + 1
FIELDSIZEY = (FIELDMAXY - FIELDMINY) + 1
FIELDSIZE = FIELDSIZEX * FIELDSIZEY
MAXSTEP = 999
FalseValue = 9999

inputX = {0: 1.0, 45: 0.5, 90: 0.0, 135: -0.5, 180: -1.0, 225: -0.5, 270: 0.0, 315: 0.5, 360: 1.0}
inputY = {0: 0.0, 45: 0.5, 90: 1.0, 135: 0.5, 180: 0.0, 225: -0.5, 270: -1.0, 315: -0.5, 360: 0.0}

outputX = {0: inputX[180], 45: inputX[225], 90: inputX[270], 135: inputX[315],
           180: inputX[0], 225: inputX[45], 270: inputX[90], 315: inputX[135]}
outputY = {0: inputY[180], 45: inputY[225], 90: inputY[270], 135: inputY[315],
           180: inputY[0], 225: inputY[45], 270: inputY[90], 315: inputY[135]}


class btr2_rcll(object):
    def __init__(self, teamName="Babytigers-R", robotNum=1, gazeboFlag=False, refbox=None):
        self.teamName = teamName
        self.robotNum = int(robotNum)
        self.gazeboFlag = bool(gazeboFlag)

        if refbox is None:
            raise RuntimeError("refbox must be provided")
        self.refbox = refbox

        # field map (distance transform)
        self.btrField = [[0 for _ in range(FIELDMINX, FIELDMAXX + 1)] for _ in range(FIELDMINY, FIELDMAXY + 1)]

        self.machineReport = MachineReportEntryBTR()
        self.prepareMachine = SendPrepareMachine.Request()

        # Kachaka
        self.kachakaIP = os.getenv("kachaka_IP")
        if not self.kachakaIP:
            raise RuntimeError("Environment variable kachaka_IP is not set")

        self.kachaka = kachaka_api.KachakaApiClient(target=self.kachakaIP + ":26400")
        self.kachaka.set_auto_homing_enabled(False)
        self.kachaka.get_battery_info()

        self.oldTheta = 90

    # --------------------------
    # Utilities
    # --------------------------
    def spin_and_beacon(self):
        """Keep refbox callbacks flowing and request updates."""
        rclpy.spin_once(self.refbox, timeout_sec=0.1)
        try:
            self.refbox.sendBeacon()
        except Exception:
            pass

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

        if int(zone) > 1000:
            x = -x

        p = Pose2D()
        p.x = float(x)
        p.y = float(y)
        p.theta = 0.0
        self.refbox.get_logger().info(f"[zoneToXY] zone={zone} -> x={p.x}, y={p.y}")
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
        """
        True if current pose is (almost) same as pose_ref.
        Using round-based quantization (safe for negatives).
        """
        self.spin_and_beacon()
        pose_now = self.kachaka_get_robot_pose("kachaka")

        def q(v: float) -> int:
            return int(round(float(v) * precision))

        same = (q(pose_ref.x) == q(pose_now.x) and
                q(pose_ref.y) == q(pose_now.y) and
                q(pose_ref.theta) == q(pose_now.theta))
        return same

    def kachaka_speak(self, text: str):
        self.spin_and_beacon()
        try:
            self.kachaka.speak(text)
        except Exception as e:
            self.refbox.get_logger().warn(f"[kachaka_speak] failed: {e}")

    def kachaka_move_to_pose(self, x: float, y: float, theta: float, start_timeout: float = 5.0, done_timeout: float = 120.0) -> bool:
        """
        Move using the SAME client (self.kachaka). No external btr2_kachaka.py.
        """
        self.spin_and_beacon()
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

        # issue command
        try:
            self.kachaka.move_to_pose(target_k.x, target_k.y, target_k.theta)
        except Exception as e:
            self.refbox.get_logger().error(f"[kachaka_move_to_pose] move_to_pose RPC failed: {e}")
            return False

        # wait start (is_command_running becomes True)
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

        # final check (close enough)
        ok = self.kachaka_stop_status(target_k, precision=50)
        self.refbox.get_logger().info(f"[kachaka_move_to_pose] finished ok={ok}")
        return ok

    # --------------------------
    # Navigation helpers
    # --------------------------
    def navToPoint(self, point: Pose2D) -> bool:
        """
        Convert grid cell (zone x,y) -> field coords -> kachaka move.
        Returns True when reached (or close enough), False otherwise.
        """
        self.kachaka_speak(f"Go to Zone {point.x} {point.y}")

        # Convert "zone coordinate" (cell center) to field continuous coordinate used by your mapping.
        px = float(point.x)
        py = float(point.y)

        if px > 0:
            px = float(int(px) - 0.5)
        else:
            px = float(int(px) + 0.5)
        py = float(int(py) - 0.5)

        # NOTE: point.theta sometimes is degree in your code. Here we assume rad (kachaka uses rad).
        # If your route theta is degree, convert before calling.
        ok = self.kachaka_move_to_pose(px, py, float(point.theta))
        self.kachaka_speak("finished")
        return ok

    def getNextPoint(self, pointNumber: int) -> Pose2D:
        route = self.refbox.refboxNavigationRoutes.route
        if not route:
            p = Pose2D()
            p.x = 0.0
            p.y = 0.0
            p.theta = 0.0
            return p

        idx = min(int(pointNumber), len(route) - 1)
        zone = route[idx].zone
        self.refbox.get_logger().info(f"[getNextPoint] idx={idx} zone={zone}")

        p = self.zoneToXY(zone)
        p.theta = 0.0  # route theta not provided; keep 0
        return p

    # --------------------------
    # Challenge Track: navigation only
    # --------------------------
    def navigation(self):
        self.refbox.get_logger().info("[navigation] start")

        # wait routes & machine info (with timeout)
        t0 = time.time()
        while (len(self.refbox.refboxNavigationRoutes.route) == 0 or
               len(self.refbox.refboxMachineInfo.machines) == 0):
            self.spin_and_beacon()
            if time.time() - t0 > 15.0:
                self.refbox.get_logger().warn("[navigation] waiting route/machine info timeout (check refbox topics/services)")
                break

        # if route is empty, nothing to do
        if len(self.refbox.refboxNavigationRoutes.route) == 0:
            self.refbox.get_logger().error("[navigation] route is empty. abort.")
            return

        # execute route points sequentially
        for i in range(len(self.refbox.refboxNavigationRoutes.route)):
            p = self.getNextPoint(i)
            self.refbox.get_logger().info(f"[navigation] go point#{i} -> ({p.x},{p.y})")
            ok = self.navToPoint(p)
            if not ok:
                self.refbox.get_logger().warn(f"[navigation] failed to reach point#{i}, retry once")
                ok2 = self.navToPoint(p)
                if not ok2:
                    self.refbox.get_logger().error(f"[navigation] abort at point#{i}")
                    break

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
        pose.theta = math.pi / 2.0

        # team color mirror
        self.spin_and_beacon()
        self.refbox.get_logger().info(f"[challenge] Team Color={self.refbox.teamColor} name={self.refbox.teamColorName}")
        if self.refbox.teamColor == 1:  # cyan?
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

    # Real robot: gazeboFlag should be False
    gazeboFlag = False

    # Setup refbox
    refbox = btr2_refbox.refbox(teamName="Babytigers-R", robotNum=robotNum, gazeboFlag=gazeboFlag)

    # wait for initial info (short)
    t0 = time.time()
    while not getattr(refbox, "refboxGameStateFlag", False):
        rclpy.spin_once(refbox, timeout_sec=0.1)
        if time.time() - t0 > 5.0:
            break

    rcll = btr2_rcll(teamName="Babytigers-R", robotNum=robotNum, gazeboFlag=gazeboFlag, refbox=refbox)
    rcll.kachaka_speak("こんにちは。btr2_rcll2025_fixed を実行中です。")

    # run requested challenge directly
    rcll.challenge(challenge_name)


if __name__ == "__main__":
    main()