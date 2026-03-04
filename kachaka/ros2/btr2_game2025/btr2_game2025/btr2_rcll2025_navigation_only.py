#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
btr2_rcll2025_navigation_only.py

目的:
- Challenge Track の Navigation を実行する
- RefBox の NavigationRoutes.route に含まれる zone を先頭から順に巡回する
- 各 zone は「zone -> (grid x,y) -> (field x,y) -> (kachaka x,y)」に変換して移動する
- 1点目だけで止まる問題を避けるため、到達判定と待ち処理を堅牢化する

実行:
export kachaka_IP=192.168.18.31
python3 btr2_rcll2025_navigation_only.py navigation 1
"""

import sys
import time
import os
import math
import subprocess
import rclpy

import btr2_refbox
import kachaka_api

from geometry_msgs.msg import Pose2D

TEAMNAME = "BabyTigers-R"

FIELDMINX = -5
FIELDMAXX = 5
FIELDMINY = 1
FIELDMAXY = 5


class BTR2RCLLNavigationOnly:
    def __init__(self, team_name: str, robot_num: int, gazebo_flag: bool, refbox):
        self.team_name = team_name
        self.robot_num = int(robot_num)
        self.gazebo_flag = bool(gazebo_flag)

        if refbox is None:
            raise RuntimeError("refbox is required")
        self.refbox = refbox

        self.kachaka_ip = os.getenv("kachaka_IP")
        if not self.kachaka_ip:
            raise RuntimeError("kachaka_IP is not set: export kachaka_IP=xxx.xxx.xxx.xxx")

        self.kachaka = kachaka_api.KachakaApiClient(target=self.kachaka_ip + ":26400")

        try:
            self.kachaka.set_auto_homing_enabled(False)
        except Exception:
            pass

        try:
            self.kachaka.get_battery_info()
        except Exception:
            pass

        self.refbox.get_logger().info(f"[init] team={self.team_name} robot={self.robot_num} kachaka_IP={self.kachaka_ip}")

    def spin_and_beacon(self, timeout_sec: float = 0.1):
        try:
            rclpy.spin_once(self.refbox, timeout_sec=timeout_sec)
        except Exception:
            pass
        try:
            self.refbox.sendBeacon()
        except Exception:
            pass

    def wait_until(self, cond_fn, timeout_sec: float, timeout_message: str) -> bool:
        t0 = time.time()
        while not cond_fn():
            self.spin_and_beacon()
            if time.time() - t0 > timeout_sec:
                self.refbox.get_logger().warn(f"[timeout] {timeout_message}")
                return False
        return True

    def ensure_rad(self, theta: float) -> float:
        th = float(theta)
        if abs(th) > 2 * math.pi and abs(th) <= 720.0:
            return math.radians(th)
        return th

    def zone_to_grid_xy(self, zone: int) -> Pose2D:
        z = abs(int(zone))
        y = float(z % 10)
        x = float((z % 100) // 10)
        if int(zone) > 1000:
            x = -x
        p = Pose2D()
        p.x = x
        p.y = y
        p.theta = 0.0
        return p

    def grid_to_field_xy(self, grid: Pose2D) -> Pose2D:
        px = float(grid.x)
        py = float(grid.y)

        if px > 0:
            px = float(int(px) - 0.5)
        else:
            px = float(int(px) + 0.5)

        py = float(int(py) - 0.5)

        p = Pose2D()
        p.x = px
        p.y = py
        p.theta = float(grid.theta)
        return p

    def field_to_kachaka(self, field_pose: Pose2D) -> Pose2D:
        team_color_name = getattr(self.refbox, "teamColorName", "C")

        k = Pose2D()
        if team_color_name == "C":
            k.x = float(field_pose.y - 0.5)
            k.y = float(-field_pose.x + 2.5)
        else:
            k.x = float(field_pose.y - 0.5)
            k.y = float(-field_pose.x + 2.5 - 5.0)

        k.theta = float(field_pose.theta - math.pi / 2.0)
        return k

    def kachaka_to_field(self, kachaka_pose: Pose2D) -> Pose2D:
        team_color_name = getattr(self.refbox, "teamColorName", "C")

        f = Pose2D()
        if team_color_name == "C":
            f.x = float(-kachaka_pose.y + 2.5)
            f.y = float(kachaka_pose.x + 0.5)
        else:
            f.x = float(-kachaka_pose.y + 2.5 - 5.0)
            f.y = float(kachaka_pose.x + 0.5)

        f.theta = float(kachaka_pose.theta + math.pi / 2.0)
        return f

    def kachaka_get_pose(self, coordinate: str) -> Pose2D:
        self.spin_and_beacon()
        pose = self.kachaka.get_robot_pose()
        if coordinate == "kachaka":
            return pose
        if coordinate == "field":
            return self.kachaka_to_field(pose)
        return self.kachaka_to_field(pose)

    @staticmethod
    def dist_xy(a: Pose2D, b: Pose2D) -> float:
        dx = float(a.x) - float(b.x)
        dy = float(a.y) - float(b.y)
        return math.sqrt(dx * dx + dy * dy)

    def close_xy(self, a: Pose2D, b: Pose2D, tol: float) -> bool:
        return self.dist_xy(a, b) <= float(tol)

    def kachaka_speak(self, text: str):
        self.spin_and_beacon()
        try:
            self.kachaka.speak(str(text))
        except Exception as e:
            self.refbox.get_logger().warn(f"[speak] failed: {e}")

    def kachaka_is_running(self) -> bool:
        try:
            return bool(self.kachaka.is_command_running())
        except Exception:
            return False

    def try_move_direct(self, target_k: Pose2D) -> bool:
        try:
            if hasattr(self.kachaka, "move_to_pose"):
                self.kachaka.move_to_pose(target_k.x, target_k.y, target_k.theta)
                return True
            if hasattr(self.kachaka, "move_to_position"):
                self.kachaka.move_to_position(target_k.x, target_k.y, target_k.theta)
                return True
        except Exception as e:
            self.refbox.get_logger().warn(f"[move] direct API error: {e}")
        return False

    def try_move_fallback(self, target_k: Pose2D) -> bool:
        cmd = ["python3", "btr2_kachaka.py", "move_to_pose", str(target_k.x), str(target_k.y), str(target_k.theta)]
        self.refbox.get_logger().warn(f"[move] fallback: {' '.join(cmd)}")
        try:
            subprocess.run(cmd, check=True)
            return True
        except Exception as e:
            self.refbox.get_logger().error(f"[move] fallback failed: {e}")
            return False

    def wait_start(self, pose_before: Pose2D, start_timeout_sec: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < start_timeout_sec:
            self.spin_and_beacon()
            if self.kachaka_is_running():
                return True
            pose_now = self.kachaka_get_pose("kachaka")
            if not self.close_xy(pose_now, pose_before, tol=0.03):
                return True
        return False

    def move_to_field_pose(
        self,
        field_x: float,
        field_y: float,
        field_theta: float,
        start_timeout_sec: float = 8.0,
        done_timeout_sec: float = 180.0,
        reach_tolerance_m: float = 0.12,
    ) -> bool:
        field_theta = self.ensure_rad(field_theta)

        target_field = Pose2D()
        target_field.x = float(field_x)
        target_field.y = float(field_y)
        target_field.theta = float(field_theta)

        target_k = self.field_to_kachaka(target_field)

        self.refbox.get_logger().info(
            f"[move] field=({target_field.x:.3f},{target_field.y:.3f},{target_field.theta:.3f}) "
            f"kachaka=({target_k.x:.3f},{target_k.y:.3f},{target_k.theta:.3f})"
        )

        now0 = self.kachaka_get_pose("kachaka")
        if self.close_xy(now0, target_k, tol=reach_tolerance_m):
            self.refbox.get_logger().info("[move] already reached")
            return True

        pose_before = self.kachaka_get_pose("kachaka")
        issued = self.try_move_direct(target_k)
        started = False

        if issued:
            started = self.wait_start(pose_before, start_timeout_sec)

        if not started:
            self.refbox.get_logger().warn("[move] not started by direct -> fallback")
            pose_before = self.kachaka_get_pose("kachaka")
            if not self.try_move_fallback(target_k):
                return False
            started = self.wait_start(pose_before, start_timeout_sec)

        if not started:
            self.refbox.get_logger().error("[move] did not start")
            return False

        t1 = time.time()
        last_log = 0.0

        while time.time() - t1 < done_timeout_sec:
            self.spin_and_beacon()

            pose_now = self.kachaka_get_pose("kachaka")
            d = self.dist_xy(pose_now, target_k)

            if d <= reach_tolerance_m:
                self.refbox.get_logger().info(f"[move] reached dist={d:.3f}")
                return True

            if time.time() - last_log > 2.0:
                last_log = time.time()
                self.refbox.get_logger().info(f"[move] running={self.kachaka_is_running()} dist={d:.3f}")

        pose_end = self.kachaka_get_pose("kachaka")
        d_end = self.dist_xy(pose_end, target_k)
        ok = d_end <= (reach_tolerance_m * 1.5)
        self.refbox.get_logger().warn(f"[move] timeout dist={d_end:.3f} ok={ok}")
        return ok

    def get_route(self):
        self.spin_and_beacon()
        try:
            return list(self.refbox.refboxNavigationRoutes.route)
        except Exception:
            return []

    def navigation(self):
        self.refbox.get_logger().info("[navigation] start")

        self.wait_until(
            lambda: len(getattr(self.refbox.refboxNavigationRoutes, "route", [])) > 0,
            20.0,
            "NavigationRoutes not received",
        )
        self.wait_until(
            lambda: len(getattr(self.refbox.refboxMachineInfo, "machines", [])) > 0,
            20.0,
            "MachineInfo not received",
        )

        route = self.get_route()
        if not route:
            self.refbox.get_logger().error("[navigation] route empty")
            return

        zones = [r.zone for r in route]
        self.refbox.get_logger().info(f"[navigation] route_len={len(route)} zones={zones}")

        i = 0
        while True:
            route = self.get_route()
            if not route:
                self.refbox.get_logger().warn("[navigation] route became empty")
                break
            if i >= len(route):
                self.refbox.get_logger().info("[navigation] finished all points")
                break

            zone = route[i].zone

            grid = self.zone_to_grid_xy(zone)
            grid.theta = 0.0

            field_pose = self.grid_to_field_xy(grid)

            self.kachaka_speak(f"point {i+1} / {len(route)} zone {zone}")
            self.refbox.get_logger().info(
                f"[navigation] point#{i} zone={zone} grid=({grid.x},{grid.y}) field=({field_pose.x},{field_pose.y})"
            )

            ok = self.move_to_field_pose(field_pose.x, field_pose.y, field_pose.theta)

            if not ok:
                self.refbox.get_logger().warn(f"[navigation] retry point#{i}")
                time.sleep(0.5)
                ok2 = self.move_to_field_pose(field_pose.x, field_pose.y, field_pose.theta)
                if not ok2:
                    self.refbox.get_logger().error(f"[navigation] abort at point#{i} zone={zone}")
                    break

            i += 1
            time.sleep(0.2)

        self.kachaka_speak("navigation finished")
        self.refbox.get_logger().info("[navigation] done")

    def run(self, challenge_name: str):
        if challenge_name != "navigation":
            self.refbox.get_logger().error(f"[run] only navigation is supported: got={challenge_name}")
            return

        pose = Pose2D()
        pose.x = -1.0 * self.robot_num - 1.5
        pose.y = 0.5
        pose.theta = math.pi / 2.0

        self.spin_and_beacon()
        team_color = getattr(self.refbox, "teamColor", 0)
        team_color_name = getattr(self.refbox, "teamColorName", "?")
        self.refbox.get_logger().info(f"[run] teamColor={team_color} teamColorName={team_color_name}")

        if team_color == 1:
            pose.x = -pose.x

        self.kachaka_speak("navigation start")
        self.move_to_field_pose(pose.x, pose.y + 1.0, pose.theta)

        self.navigation()


def main():
    rclpy.init(args=None)

    argv = sys.argv
    if len(argv) < 2:
        print("Usage: python3 btr2_rcll2025_navigation_only.py navigation [robotNum]")
        return

    challenge_name = argv[1]
    robot_num = int(argv[2]) if len(argv) >= 3 else 1
    gazebo_flag = False

    refbox = btr2_refbox.refbox(teamName=TEAMNAME, robotNum=robot_num, gazeboFlag=gazebo_flag)

    node = BTR2RCLLNavigationOnly(team_name=TEAMNAME, robot_num=robot_num, gazebo_flag=gazebo_flag, refbox=refbox)
    node.kachaka_speak("btr2_rcll2025_navigation_only start")

    node.run(challenge_name)

    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()