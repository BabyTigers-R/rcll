#!/usr/bin/env python3
import sys
import time
import os
import math
import subprocess
import threading
import rclpy

import btr2_refbox
import kachaka_api

from geometry_msgs.msg import Pose2D
from refbox_msgs.msg import MachineReportEntryBTR
from refbox_msgs.srv import SendPrepareMachine

TEAMNAME = "BabyTigers-R"


class Btr2Rcll:
    def __init__(self, team_name: str, robot_num: int, gazebo_flag: bool, refbox):
        self.team_name = team_name
        self.robot_num = int(robot_num)
        self.gazebo_flag = bool(gazebo_flag)
        self.refbox = refbox

        self.machineReport = MachineReportEntryBTR()
        self.prepareMachine = SendPrepareMachine.Request()

        self.kachaka_ip = os.getenv("kachaka_IP")
        if not self.kachaka_ip:
            raise RuntimeError("Environment variable kachaka_IP is not set")

        self.kachaka = kachaka_api.KachakaApiClient(target=self.kachaka_ip + ":26400")
        self.kachaka.set_auto_homing_enabled(False)
        self.kachaka.get_battery_info()

        self.refbox.get_logger().info(
            f"[init] team={self.team_name} robot={self.robot_num} kachaka_IP={self.kachaka_ip}"
        )

    def spin_and_beacon(self, timeout_sec: float = 0.1):
        try:
            rclpy.spin_once(self.refbox, timeout_sec=timeout_sec)
        except Exception:
            pass
        try:
            self.refbox.sendBeacon()
        except Exception:
            pass

    def wait_until(self, cond_fn, timeout_sec: float, label: str) -> bool:
        t0 = time.time()
        while True:
            if cond_fn():
                return True
            self.spin_and_beacon()
            if time.time() - t0 >= timeout_sec:
                self.refbox.get_logger().warn(f"[timeout] {label}")
                return False

    def ensure_rad(self, theta: float) -> float:
        th = float(theta)
        if abs(th) > 2 * math.pi and abs(th) <= 720.0:
            return math.radians(th)
        return th

    def normalize_rad(self, theta: float) -> float:
        th = float(theta)
        while th > math.pi:
            th -= 2.0 * math.pi
        while th <= -math.pi:
            th += 2.0 * math.pi
        return th

    def _close_xy(self, a: Pose2D, b: Pose2D, tol: float = 0.10) -> bool:
        dx = float(a.x) - float(b.x)
        dy = float(a.y) - float(b.y)
        return (dx * dx + dy * dy) ** 0.5 <= tol

    def _close_pose(self, a: Pose2D, b: Pose2D, xy_tol: float = 0.12, th_tol: float = 0.35) -> bool:
        if not self._close_xy(a, b, tol=xy_tol):
            return False
        dth = self.normalize_rad(float(a.theta) - float(b.theta))
        return abs(dth) <= th_tol

    # ------------------------------------------------------------
    # 旧コード準拠: zone -> grid coordinate
    #   31   -> ( 3, 1)
    #   1031 -> (-3, 1)
    # ------------------------------------------------------------
    def zone_to_xy(self, zone: int) -> Pose2D:
        z = abs(int(zone))
        p = Pose2D()
        p.y = float(z % 10)
        p.x = float((z % 100) // 10)
        if int(zone) > 1000:
            p.x = -p.x
        p.theta = 0.0
        return p

    # ------------------------------------------------------------
    # 旧コード navToPoint と同じ変換
    #   grid x= 3 -> field x= 2.5
    #   grid x=-3 -> field x=-2.5
    #   grid y= 1 -> field y= 0.5
    # ------------------------------------------------------------
    def zone_center_to_field_xy(self, zone_x: float, zone_y: float) -> tuple[float, float]:
        if zone_x > 0:
            px = float(int(zone_x) - 0.5)
        else:
            px = float(int(zone_x) + 0.5)
        py = float(int(zone_y) - 0.5)
        return px, py

    # ------------------------------------------------------------
    # 旧コードと完全に同じ field <-> kachaka 変換
    # ------------------------------------------------------------
    def field2kachaka(self, pose: Pose2D) -> Pose2D:
        k = Pose2D()
        if getattr(self.refbox, "teamColorName", "") == "C":
            k.x = float(pose.y - 0.5)
            k.y = float(-pose.x + 2.5)
        else:
            k.x = float(pose.y - 0.5)
            k.y = float(-pose.x + 2.5 - 5.0)
        k.theta = self.normalize_rad(float(pose.theta - math.pi / 2.0))
        return k

    def kachaka2field(self, pose: Pose2D) -> Pose2D:
        f = Pose2D()
        if getattr(self.refbox, "teamColorName", "") == "C":
            f.x = float(-pose.y + 2.5)
            f.y = float(pose.x + 0.5)
        else:
            f.x = float(-pose.y + 2.5 - 5.0)
            f.y = float(pose.x + 0.5)
        f.theta = self.normalize_rad(float(pose.theta + math.pi / 2.0))
        return f

    def kachaka_get_robot_pose(self, coordinate: str) -> Pose2D:
        pose = self.kachaka.get_robot_pose()
        if coordinate == "kachaka":
            return pose
        return self.kachaka2field(pose)

    def kachaka_speak(self, text: str):
        self.spin_and_beacon()
        try:
            self.kachaka.speak(text)
        except Exception as e:
            self.refbox.get_logger().warn(f"[speak] {e}")

    def _start_kachaka_move_thread(self, target_k: Pose2D):
        result = {
            "ok": False,
            "error": None,
            "started_api": None,
        }

        def worker():
            try:
                if hasattr(self.kachaka, "move_to_pose"):
                    self.kachaka.move_to_pose(target_k.x, target_k.y, target_k.theta)
                    result["ok"] = True
                    result["started_api"] = "move_to_pose"
                    return

                if hasattr(self.kachaka, "move_to_position"):
                    self.kachaka.move_to_position(target_k.x, target_k.y, target_k.theta)
                    result["ok"] = True
                    result["started_api"] = "move_to_position"
                    return

                result["error"] = "No move_to_pose / move_to_position API"
            except Exception as e:
                result["error"] = str(e)

        th = threading.Thread(target=worker, daemon=True)
        th.start()
        return th, result

    def _start_fallback_move_thread(self, target_k: Pose2D):
        result = {
            "ok": False,
            "error": None,
        }

        def worker():
            cmd = [
                "python3",
                "btr2_kachaka.py",
                "move_to_pose",
                str(target_k.x),
                str(target_k.y),
                str(target_k.theta),
            ]
            try:
                self.refbox.get_logger().warn(f"[move] fallback cmd={' '.join(cmd)}")
                subprocess.run(cmd, check=True, env=os.environ.copy())
                result["ok"] = True
            except Exception as e:
                result["error"] = str(e)

        th = threading.Thread(target=worker, daemon=True)
        th.start()
        return th, result

    def _wait_move_start(self, prev_pose: Pose2D, start_timeout: float = 8.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < start_timeout:
            self.spin_and_beacon()

            try:
                if self.kachaka.is_command_running():
                    self.refbox.get_logger().info("[move] detected command running")
                    return True
            except Exception:
                pass

            try:
                nowp = self.kachaka_get_robot_pose("kachaka")
                if not self._close_xy(nowp, prev_pose, tol=0.03):
                    self.refbox.get_logger().info("[move] detected pose change")
                    return True
            except Exception:
                pass

            time.sleep(0.05)

        return False

    def _wait_move_done(self, target_k: Pose2D, done_timeout: float = 180.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < done_timeout:
            self.spin_and_beacon()

            try:
                nowp = self.kachaka_get_robot_pose("kachaka")
                if self._close_pose(nowp, target_k, xy_tol=0.12, th_tol=0.40):
                    self.refbox.get_logger().info("[move] reached")
                    return True
            except Exception:
                pass

            try:
                if not self.kachaka.is_command_running():
                    time.sleep(0.2)
                    nowp2 = self.kachaka_get_robot_pose("kachaka")
                    if self._close_pose(nowp2, target_k, xy_tol=0.15, th_tol=0.50):
                        self.refbox.get_logger().info("[move] reached(after stop)")
                        return True
            except Exception:
                pass

            time.sleep(0.05)

        self.refbox.get_logger().warn("[move] timeout while waiting done")
        try:
            nowp = self.kachaka_get_robot_pose("kachaka")
            ok = self._close_pose(nowp, target_k, xy_tol=0.18, th_tol=0.60)
            self.refbox.get_logger().info(f"[move] final pose check ok={ok}")
            return ok
        except Exception:
            return False

    def kachaka_move_to_pose(
        self,
        x: float,
        y: float,
        theta: float,
        start_timeout: float = 8.0,
        done_timeout: float = 180.0,
    ) -> bool:
        theta = self.normalize_rad(self.ensure_rad(theta))

        self.refbox.get_logger().info(f"[move] field=({x:.3f},{y:.3f},{theta:.3f})")

        target_field = Pose2D()
        target_field.x = float(x)
        target_field.y = float(y)
        target_field.theta = float(theta)

        target_k = self.field2kachaka(target_field)
        self.refbox.get_logger().info(
            f"[move] kachaka=({target_k.x:.3f},{target_k.y:.3f},{target_k.theta:.3f})"
        )

        try:
            pose_now0 = self.kachaka_get_robot_pose("kachaka")
            if self._close_pose(pose_now0, target_k, xy_tol=0.10, th_tol=0.35):
                self.refbox.get_logger().info("[move] already reached")
                return True
        except Exception as e:
            self.refbox.get_logger().warn(f"[move] initial pose read failed: {e}")

        prev_pose = self.kachaka_get_robot_pose("kachaka")
        move_thread, move_result = self._start_kachaka_move_thread(target_k)

        started = self._wait_move_start(prev_pose, start_timeout=start_timeout)

        if started:
            self.refbox.get_logger().info(
                f"[move] started by direct API ({move_result.get('started_api')})"
            )
            return self._wait_move_done(target_k, done_timeout=done_timeout)

        if not move_thread.is_alive() and not move_result["ok"]:
            self.refbox.get_logger().warn(f"[move] direct API failed: {move_result['error']}")
        else:
            self.refbox.get_logger().warn("[move] direct API did not start movement")

        self.refbox.get_logger().warn("[move] try fallback in background thread")
        prev_pose = self.kachaka_get_robot_pose("kachaka")
        fb_thread, fb_result = self._start_fallback_move_thread(target_k)

        started = self._wait_move_start(prev_pose, start_timeout=start_timeout)

        if not started:
            if not fb_thread.is_alive() and not fb_result["ok"]:
                self.refbox.get_logger().error(f"[move] fallback failed: {fb_result['error']}")
            else:
                self.refbox.get_logger().error("[move] fallback did not start")
            return False

        self.refbox.get_logger().info("[move] started by fallback")
        return self._wait_move_done(target_k, done_timeout=done_timeout)

    def get_route(self):
        self.spin_and_beacon()
        try:
            return list(self.refbox.refboxNavigationRoutes.route)
        except Exception:
            return []

    def direction_to_theta(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return self.normalize_rad(math.atan2(y2 - y1, x2 - x1))

    def navigation(self):
        self.refbox.get_logger().info("[navigation] start")

        ok_route = self.wait_until(
            lambda: len(self.get_route()) > 0,
            20.0,
            "NavigationRoutes not received",
        )
        ok_mps = self.wait_until(
            lambda: len(getattr(self.refbox, "refboxMachineInfo", type("X", (), {"machines": []})()).machines) > 0,
            20.0,
            "MachineInfo not received",
        )

        if not ok_route:
            self.refbox.get_logger().error("[navigation] route empty")
            return
        if not ok_mps:
            self.refbox.get_logger().warn("[navigation] MachineInfo missing (continue)")

        route_snapshot = self.get_route()
        if not route_snapshot:
            self.refbox.get_logger().error("[navigation] route empty (snapshot)")
            return

        zones = [int(r.zone) for r in route_snapshot]
        self.refbox.get_logger().info(f"[navigation] route_len={len(zones)} zones={zones}")

        prev_theta = math.pi / 2.0

        for i, zone in enumerate(zones):
            p_zone = self.zone_to_xy(zone)
            px, py = self.zone_center_to_field_xy(p_zone.x, p_zone.y)

            if i + 1 < len(zones):
                p_next = self.zone_to_xy(zones[i + 1])
                nx, ny = self.zone_center_to_field_xy(p_next.x, p_next.y)
                theta = self.direction_to_theta(px, py, nx, ny)
                prev_theta = theta
            else:
                theta = prev_theta

            self.refbox.get_logger().info(
                f"[navigation] {i+1}/{len(zones)} "
                f"zone={zone} zone_xy=({p_zone.x:.1f},{p_zone.y:.1f}) "
                f"field_xy=({px:.3f},{py:.3f}) theta={theta:.3f}"
            )

            moved = self.kachaka_move_to_pose(px, py, theta)

            if not moved:
                self.refbox.get_logger().warn(f"[navigation] move failed zone={zone} -> retry once")
                if not self.kachaka_move_to_pose(px, py, theta):
                    self.refbox.get_logger().error(f"[navigation] abort at zone={zone}")
                    return

            self.refbox.get_logger().info(f"[navigation] reached zone={zone}")

        self.refbox.get_logger().info("[navigation] done")

    def challenge(self, name: str):
        if name != "navigation":
            self.refbox.get_logger().error(f"[challenge] unknown: {name}")
            return

        pose = Pose2D()
        pose.x = -1.0 * self.robot_num - 1.5
        pose.y = 0.5
        pose.theta = math.pi / 2.0

        self.spin_and_beacon()
        team_color = getattr(self.refbox, "teamColor", 0)
        team_color_name = getattr(self.refbox, "teamColorName", "")
        self.refbox.get_logger().info(f"[run] teamColor={team_color} teamColorName={team_color_name}")

        if team_color == 1:
            pose.x = -pose.x

        start_x = pose.x
        start_y = pose.y
        start_theta = pose.theta

        self.kachaka_speak("navigation start")
        self.kachaka_move_to_pose(start_x, start_y, start_theta)

        self.navigation()


def main():
    rclpy.init(args=None)

    if len(sys.argv) < 2:
        print("Usage: python3 btr2_rcll2025_navigation_only.py navigation [robotNum]")
        return

    challenge_name = sys.argv[1]
    robot_num = int(sys.argv[2]) if len(sys.argv) >= 3 else 1

    gazebo_flag = False
    refbox = btr2_refbox.refbox(
        teamName=TEAMNAME,
        robotNum=robot_num,
        gazeboFlag=gazebo_flag,
    )

    node = Btr2Rcll(
        team_name=TEAMNAME,
        robot_num=robot_num,
        gazebo_flag=gazebo_flag,
        refbox=refbox,
    )

    node.kachaka_speak("btr2 navigation only")
    node.challenge(challenge_name)

    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()