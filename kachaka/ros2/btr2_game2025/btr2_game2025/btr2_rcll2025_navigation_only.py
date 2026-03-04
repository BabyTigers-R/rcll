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

    def _close_xy(self, a: Pose2D, b: Pose2D, tol: float = 0.10) -> bool:
        dx = float(a.x) - float(b.x)
        dy = float(a.y) - float(b.y)
        return (dx * dx + dy * dy) ** 0.5 <= tol

    def zone_to_xy(self, zone: int) -> Pose2D:
        z = abs(int(zone))
        y = float(z % 10)
        x = float((z % 100) // 10)
        if int(zone) > 1000:
            x = -x
        p = Pose2D()
        p.x = float(x)
        p.y = float(y)
        p.theta = 0.0
        return p

    def field2kachaka(self, pose: Pose2D) -> Pose2D:
        k = Pose2D()
        if getattr(self.refbox, "teamColorName", "") == "C":
            k.x = float(pose.y - 0.5)
            k.y = float(-pose.x + 2.5)
        else:
            k.x = float(pose.y - 0.5)
            k.y = float(-pose.x + 2.5 - 5.0)
        k.theta = float(pose.theta - math.pi / 2.0)
        return k

    def kachaka2field(self, pose: Pose2D) -> Pose2D:
        f = Pose2D()
        if getattr(self.refbox, "teamColorName", "") == "C":
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
        return self.kachaka2field(pose)

    def kachaka_speak(self, text: str):
        self.spin_and_beacon()
        try:
            self.kachaka.speak(text)
        except Exception as e:
            self.refbox.get_logger().warn(f"[speak] {e}")

    def kachaka_move_to_pose(
        self,
        x: float,
        y: float,
        theta: float,
        start_timeout: float = 8.0,
        done_timeout: float = 180.0,
    ) -> bool:
        self.spin_and_beacon()
        theta = self.ensure_rad(theta)

        self.refbox.get_logger().info(f"[move] field=({x:.3f},{y:.3f},{theta:.3f})")

        target_field = Pose2D()
        target_field.x = float(x)
        target_field.y = float(y)
        target_field.theta = float(theta)

        target_k = self.field2kachaka(target_field)
        self.refbox.get_logger().info(f"[move] kachaka=({target_k.x:.3f},{target_k.y:.3f},{target_k.theta:.3f})")

        pose_now0 = self.kachaka_get_robot_pose("kachaka")
        if self._close_xy(pose_now0, target_k, tol=0.10):
            self.refbox.get_logger().info("[move] already reached")
            return True

        def try_direct() -> bool:
            try:
                if hasattr(self.kachaka, "move_to_pose"):
                    self.kachaka.move_to_pose(target_k.x, target_k.y, target_k.theta)
                    return True
                if hasattr(self.kachaka, "move_to_position"):
                    self.kachaka.move_to_position(target_k.x, target_k.y, target_k.theta)
                    return True
            except Exception as e:
                self.refbox.get_logger().warn(f"[move] direct exception: {e}")
            return False

        def try_fallback() -> bool:
            cmd = ["python3", "btr2_kachaka.py", "move_to_pose",
                   str(target_k.x), str(target_k.y), str(target_k.theta)]
            self.refbox.get_logger().warn(f"[move] fallback cmd={' '.join(cmd)}")
            try:
                subprocess.run(cmd, check=True)
                return True
            except Exception as e:
                self.refbox.get_logger().error(f"[move] fallback failed: {e}")
                return False

        def wait_start(prev_pose: Pose2D) -> bool:
            t0 = time.time()
            while time.time() - t0 < start_timeout:
                self.spin_and_beacon()
                try:
                    if self.kachaka.is_command_running():
                        return True
                except Exception:
                    pass
                nowp = self.kachaka_get_robot_pose("kachaka")
                if not self._close_xy(nowp, prev_pose, tol=0.03):
                    return True
            return False

        prev_pose = self.kachaka_get_robot_pose("kachaka")
        direct_ok = try_direct()

        started = False
        if direct_ok:
            started = wait_start(prev_pose)

        if not started:
            self.refbox.get_logger().warn("[move] direct did not start -> fallback")
            prev_pose = self.kachaka_get_robot_pose("kachaka")
            if not try_fallback():
                return False
            started = wait_start(prev_pose)

        if not started:
            self.refbox.get_logger().error("[move] did not start")
            return False

        t1 = time.time()
        while time.time() - t1 < done_timeout:
            self.spin_and_beacon()

            nowp = self.kachaka_get_robot_pose("kachaka")
            if self._close_xy(nowp, target_k, tol=0.12):
                self.refbox.get_logger().info("[move] reached")
                return True

            try:
                if not self.kachaka.is_command_running():
                    time.sleep(0.2)
                    nowp2 = self.kachaka_get_robot_pose("kachaka")
                    if self._close_xy(nowp2, target_k, tol=0.12):
                        self.refbox.get_logger().info("[move] reached(after stop)")
                        return True
            except Exception:
                pass

        self.refbox.get_logger().warn("[move] timeout")
        nowp = self.kachaka_get_robot_pose("kachaka")
        ok = self._close_xy(nowp, target_k, tol=0.15)
        self.refbox.get_logger().info(f"[move] finished ok={ok}")
        return ok

    def zone_center_to_field_xy(self, zone_x: float, zone_y: float) -> tuple[float, float]:
        px = float(zone_x)
        py = float(zone_y)
        if px > 0:
            px = float(int(px) - 0.5)
        else:
            px = float(int(px) + 0.5)
        py = float(int(py) - 0.5)
        return px, py

    def get_route(self):
        self.spin_and_beacon()
        try:
            return list(self.refbox.refboxNavigationRoutes.route)
        except Exception:
            return []

    def navigation(self):
        self.refbox.get_logger().info("[navigation] start")

        ok_route = self.wait_until(lambda: len(self.get_route()) > 0, 20.0, "NavigationRoutes not received")
        ok_mps = self.wait_until(lambda: len(getattr(self.refbox.refboxMachineInfo, "machines", [])) > 0, 20.0, "MachineInfo not received")
        if not ok_route:
            self.refbox.get_logger().error("[navigation] route empty")
            return
        if not ok_mps:
            self.refbox.get_logger().warn("[navigation] MachineInfo missing (continue)")

        last_head_zone = None

        while True:
            route = self.get_route()
            if not route:
                self.refbox.get_logger().info("[navigation] route empty -> done")
                return

            head_zone = int(route[0].zone)

            if last_head_zone is None:
                last_head_zone = head_zone

            p_zone = self.zone_to_xy(head_zone)
            px, py = self.zone_center_to_field_xy(p_zone.x, p_zone.y)

            self.refbox.get_logger().info(f"[navigation] next zone={head_zone} -> zone_xy=({p_zone.x},{p_zone.y}) field_xy=({px:.3f},{py:.3f})")

            self.kachaka_speak(f"next {head_zone}")

            moved = self.kachaka_move_to_pose(px, py, 0.0)

            if not moved:
                self.refbox.get_logger().warn("[navigation] move failed -> retry once")
                if not self.kachaka_move_to_pose(px, py, 0.0):
                    self.refbox.get_logger().error("[navigation] abort")
                    return

            self.refbox.get_logger().info("[navigation] reached -> wait route update")

            t0 = time.time()
            while True:
                self.spin_and_beacon()
                route2 = self.get_route()
                if not route2:
                    self.refbox.get_logger().info("[navigation] route empty -> done")
                    return

                new_head = int(route2[0].zone)

                if new_head != head_zone:
                    last_head_zone = new_head
                    break

                if time.time() - t0 > 15.0:
                    self.refbox.get_logger().warn("[navigation] route did not update, continue anyway")
                    break

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

        px, py = pose.x, pose.y + 1.0
        self.kachaka_speak("navigation start")
        self.kachaka_move_to_pose(px, py, pose.theta)

        self.navigation()


def main():
    rclpy.init(args=None)

    if len(sys.argv) < 2:
        print("Usage: python3 btr2_rcll2025_navigation_only.py navigation [robotNum]")
        return

    challenge_name = sys.argv[1]
    robot_num = int(sys.argv[2]) if len(sys.argv) >= 3 else 1

    gazebo_flag = False
    refbox = btr2_refbox.refbox(teamName=TEAMNAME, robotNum=robot_num, gazeboFlag=gazebo_flag)

    node = Btr2Rcll(team_name=TEAMNAME, robot_num=robot_num, gazebo_flag=gazebo_flag, refbox=refbox)
    node.kachaka_speak("btr2 navigation only")

    node.challenge(challenge_name)

    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()