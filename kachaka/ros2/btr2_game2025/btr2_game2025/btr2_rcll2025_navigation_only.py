#!/usr/bin/python3
import sys
import subprocess
import rclpy
import btr2_refbox
import kachaka_api
import os
import math
import time

from geometry_msgs.msg import Pose2D
from refbox_msgs.msg import MachineReportEntryBTR
from refbox_msgs.srv import SendPrepareMachine

TEAMNAME = "BabyTigers-R"

### for Challenge Track
FIELDMINX = -5
FIELDMAXX = 5
FIELDMINY = 1
FIELDMAXY = 5

### for Main Track
# FIELDMINX = -7
# FIELDMAXX = 7
# FIELDMINY = 1
# FIELDMAXY = 8

FIELDSIZEX = (FIELDMAXX - FIELDMINX) + 1
FIELDSIZEY = (FIELDMAXY - FIELDMINY) + 1
FIELDSIZE = FIELDSIZEX * FIELDSIZEY
MAXSTEP = 999
FalseValue = 9999

inputX = {0: 1.0, 45: 0.5, 90: 0, 135: -0.5, 180: -1.0, 225: -0.5, 270: 0, 315: 0.5, 360: 1.0}
inputY = {0: 0, 45: 0.5, 90: 1.0, 135: 0.5, 180: 0, 225: -0.5, 270: -1.0, 315: -0.5, 360: 0}

outputX = {
    0: inputX[180], 45: inputX[225], 90: inputX[270], 135: inputX[315],
    180: inputX[0], 225: inputX[45], 270: inputX[90], 315: inputX[135]
}
outputY = {
    0: inputY[180], 45: inputY[225], 90: inputY[270], 135: inputY[315],
    180: inputY[0], 225: inputY[45], 270: inputY[90], 315: inputY[135]
}

CS_OP_RETRIEVE_CAP = 1
CS_OP_MOUNT_CAP = 2
BS_SIDE_INPUT = 1
BS_SIDE_OUTPUT = 2


class btr2_rcll(object):
    def __init__(self, teamName="Babytigers-R", robotNum=0, gazeboFlag=False, refbox=None):
        self.topicName = ""
        self.gazeboFlag = gazeboFlag
        self.robotNum = robotNum

        if gazeboFlag:
            self.topicName = "/btr-robot" + str(robotNum)

        if refbox is None:
            print("please set refbox arg")
            sys.exit(1)

        self.refbox = refbox
        self.btrField = [[0 for _ in range(FIELDMINX, FIELDMAXX + 1)] for _ in range(FIELDMINY, FIELDMAXY + 1)]
        self.machineReport = MachineReportEntryBTR()
        self.prepareMachine = SendPrepareMachine.Request()

        self.kachakaIP = os.getenv("kachaka_IP")
        if not self.kachakaIP:
            raise RuntimeError("Environment variable kachaka_IP is not set")

        self.kachaka = kachaka_api.KachakaApiClient(target=self.kachakaIP + ":26400")
        self.kachaka.set_auto_homing_enabled(False)
        self.kachaka.get_battery_info()

        self.oldTheta = math.pi / 2.0

    # ------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------
    def spin_and_beacon(self, timeout_sec=0.1):
        try:
            rclpy.spin_once(self.refbox, timeout_sec=timeout_sec)
        except Exception:
            pass
        try:
            self.refbox.sendBeacon()
        except Exception:
            pass

    def normalize_rad(self, theta):
        th = float(theta)
        while th > math.pi:
            th -= 2.0 * math.pi
        while th <= -math.pi:
            th += 2.0 * math.pi
        return th

    def _close_xy(self, a, b, tol=0.10):
        dx = float(a.x) - float(b.x)
        dy = float(a.y) - float(b.y)
        return math.hypot(dx, dy) <= tol

    def _close_pose(self, a, b, xy_tol=0.12, th_tol=0.35):
        if not self._close_xy(a, b, tol=xy_tol):
            return False
        dth = self.normalize_rad(float(a.theta) - float(b.theta))
        return abs(dth) <= th_tol

    def safe_get_robot_pose(self, coordinate="kachaka", retry=3, sleep_sec=0.05):
        for _ in range(retry):
            try:
                return self.kachaka_get_robot_pose(coordinate)
            except Exception:
                time.sleep(sleep_sec)
        raise RuntimeError(f"failed to get robot pose in {coordinate}")

    # ------------------------------------------------------------
    # Field helper
    # ------------------------------------------------------------
    def setField(self, x, y, number):
        if x < FIELDMINX or x > FIELDMAXX or y < FIELDMINY or y > FIELDMAXY:
            print("setField - out of field: ", x, y, number)
            return
        self.btrField[int(y) - FIELDMINY][int(x) - FIELDMINX] = number

    def getField(self, x, y):
        if int(x) < FIELDMINX or FIELDMAXX < int(x) or int(y) < FIELDMINY or FIELDMAXY < int(y):
            return MAXSTEP
        return self.btrField[int(y) - FIELDMINY][int(x) - FIELDMINX]

    def getStep(self, x, y):
        if ((x < FIELDMINX or FIELDMAXX < x) or (y < FIELDMINY or FIELDMAXY < y)):
            return MAXSTEP

        step = self.getField(x, y)
        if step == 0:
            step = MAXSTEP
        return step

    def zoneToXY(self, zone):
        point = Pose2D()
        point.y = float(abs(int(zone)) % 10)
        point.x = float((abs(int(zone)) % 100) // 10)
        if int(zone) > 1000:
            point.x = -point.x
        point.theta = 0.0
        print(zone, point.x, point.y)
        return point

    def zone_center_to_field_xy(self, zone_x, zone_y):
        if zone_x > 0:
            px = float(int(zone_x) - 0.5)
        else:
            px = float(int(zone_x) + 0.5)
        py = float(int(zone_y) - 0.5)
        return px, py

    def direction_to_theta(self, x1, y1, x2, y2):
        return self.normalize_rad(math.atan2(y2 - y1, x2 - x1))

    # ------------------------------------------------------------
    # Wall / path
    # ------------------------------------------------------------
    def wallCheck(self, x, y, dx, dy):
        notWallFlag = True
        if FIELDMINX == -5:
            if ((x == -5 and y == 1) and (dx == 0 and dy == 1)):
                notWallFlag = False
            if ((x == -4 and y == 1) and (dx == 0 and dy == 1)):
                notWallFlag = False
            if ((x == -3 and y == 1) and (dx == 1 and dy == 0)):
                notWallFlag = False
            if ((x == -2 and y == 1) and (dx == -1 and dy == 0)):
                notWallFlag = False
            if ((x == -5 and y == 2) and (dx == 0 and dy == -1)):
                notWallFlag = False
            if ((x == -4 and y == 2) and (dx == 0 and dy == -1)):
                notWallFlag = False

            if ((x == 5 and y == 1) and (dx == 0 and dy == 1)):
                notWallFlag = False
            if ((x == 4 and y == 1) and (dx == 0 and dy == 1)):
                notWallFlag = False
            if ((x == 3 and y == 1) and (dx == -1 and dy == 0)):
                notWallFlag = False
            if ((x == 2 and y == 1) and (dx == 1 and dy == 0)):
                notWallFlag = False
            if ((x == 5 and y == 2) and (dx == 0 and dy == -1)):
                notWallFlag = False
            if ((x == 4 and y == 2) and (dx == 0 and dy == -1)):
                notWallFlag = False
        else:
            if ((x == -6 and y == 1) and (dx == 0 and dy == 1)):
                notWallFlag = False
            if ((x == -7 and y == 1) and (dx == 0 and dy == 1)):
                notWallFlag = False
            if ((x == -5 and y == 1) and (dx == 1 and dy == 0)):
                notWallFlag = False
            if ((x == -4 and y == 1) and (dx == -1 and dy == 0)):
                notWallFlag = False
            if ((x == -6 and y == 2) and (dx == 0 and dy == -1)):
                notWallFlag = False

            if ((x == 6 and y == 1) and (dx == 0 and dy == 1)):
                notWallFlag = False
            if ((x == 7 and y == 1) and (dx == 0 and dy == 1)):
                notWallFlag = False
            if ((x == 5 and y == 1) and (dx == -1 and dy == 0)):
                notWallFlag = False
            if ((x == 4 and y == 1) and (dx == 1 and dy == 0)):
                notWallFlag = False
            if ((x == 6 and y == 2) and (dx == 0 and dy == -1)):
                notWallFlag = False

        if x < FIELDMINX or x > FIELDMAXX or y < FIELDMINY or y > FIELDMAXY:
            notWallFlag = False

        return notWallFlag

    def getNextDirection(self, x, y):
        minStep = self.getField(x, y)
        nextD = Pose2D()
        nextD.x = 0.0
        nextD.y = 0.0
        for dx, dy in zip([-1, 1, 0, 0], [0, 0, -1, 1]):
            notWallFlag = self.wallCheck(x, y, dx, dy)
            print(x, y, dx, dy, notWallFlag)
            if (minStep > self.getField(x + dx, y + dy)) and (notWallFlag is True):
                minStep = self.getField(x + dx, y + dy)
                nextD.x = float(dx)
                nextD.y = float(dy)
                print("nextDirection", nextD.x, nextD.y, "now: ", self.getField(x, y), "next: ", self.getField(x + nextD.x, y + nextD.y))
        return nextD

    # ------------------------------------------------------------
    # Kachaka coordinate conversion
    # ------------------------------------------------------------
    def field2kachaka(self, pose):
        kachaka_pose = Pose2D()
        if self.refbox.teamColorName == "C":
            kachaka_pose.x = float(pose.y - 0.5)
            kachaka_pose.y = float(-pose.x + 2.5)
        else:
            kachaka_pose.x = float(pose.y - 0.5)
            kachaka_pose.y = float(-pose.x + 2.5 - 5.0)
        kachaka_pose.theta = self.normalize_rad(float(pose.theta - math.pi / 2.0))
        return kachaka_pose

    def kachaka2field(self, pose):
        field_pose = Pose2D()
        if self.refbox.teamColorName == "C":
            field_pose.x = float(-pose.y + 2.5)
            field_pose.y = float(pose.x + 0.5)
        else:
            field_pose.x = float(-pose.y + 2.5 - 5.0)
            field_pose.y = float(pose.x + 0.5)
        field_pose.theta = self.normalize_rad(float(pose.theta + math.pi / 2.0))
        return field_pose

    def kachaka_get_robot_pose(self, coordinate):
        self.spin_and_beacon()
        pose = self.kachaka.get_robot_pose()
        if coordinate == "field":
            return self.kachaka2field(pose)
        if coordinate == "kachaka":
            return pose
        print(f"[kachaka_get_robot_pose] unknown coordinate {coordinate}")
        return self.kachaka2field(pose)

    def kachaka_speak(self, data):
        self.spin_and_beacon()
        try:
            self.kachaka.speak(data)
        except Exception as e:
            self.refbox.get_logger().warn(f"[speak] {e}")

    def kachaka_stop_status(self, target_pose):
        self.spin_and_beacon()
        now_pose = self.kachaka_get_robot_pose("kachaka")

        dx = float(now_pose.x) - float(target_pose.x)
        dy = float(now_pose.y) - float(target_pose.y)
        dth = self.normalize_rad(float(now_pose.theta) - float(target_pose.theta))

        pos_ok = math.hypot(dx, dy) < 0.10
        th_ok = abs(dth) < 0.20
        return pos_ok and th_ok

    def launch_kachaka_move(self, kachaka_pose):
        cmd = (
            f"export kachaka_IP={self.kachakaIP}; "
            f"python3 btr2_kachaka.py move_to_pose {kachaka_pose.x} {kachaka_pose.y} {kachaka_pose.theta} > /dev/null 2>&1 &"
        )
        self.refbox.get_logger().info(cmd)
        os.system(cmd)

    def wait_for_motion_start(self, prev_pose, timeout_sec=8.0):
        start_t = time.time()
        while time.time() - start_t < timeout_sec:
            self.spin_and_beacon()
            try:
                if self.kachaka.is_command_running():
                    return True
            except Exception:
                pass

            try:
                nowp = self.safe_get_robot_pose("kachaka")
                if not self._close_xy(nowp, prev_pose, tol=0.03):
                    return True
            except Exception:
                pass

            time.sleep(0.05)
        return False

    def kachaka_move_to_pose(self, x, y, theta):
        self.spin_and_beacon()
        self.refbox.get_logger().info(f"[kachaka_move_to_pose in the field]: ({x}, {y}, {theta})")

        pose = Pose2D()
        pose.x = float(x)
        pose.y = float(y)
        pose.theta = float(theta)

        kachaka = self.field2kachaka(pose)

        try:
            running = self.kachaka.is_command_running()
            self.refbox.get_logger().info(f"running? {running}")
        except Exception:
            pass

        try:
            pose_now = self.safe_get_robot_pose("kachaka")
            self.refbox.get_logger().info(f"pose_now: {pose_now}")
        except Exception:
            pass

        if self.kachaka_stop_status(kachaka):
            print("[kachaka_move_to_pose] already arrived")
            return True

        print(f"[kachaka_move_to_pose in kachaka]: ({kachaka.x}, {kachaka.y}, {kachaka.theta})")
        self.launch_kachaka_move(kachaka)
        print("[kachaka_move_to_pose] wait for move")

        try:
            prev_pose = self.safe_get_robot_pose("kachaka")
        except Exception:
            prev_pose = kachaka

        self.wait_for_motion_start(prev_pose, timeout_sec=8.0)

        print(f"[kachaka_move_to_pose] running to {pose} in field")

        done_t = time.time()
        while time.time() - done_t < 180.0:
            self.spin_and_beacon()
            try:
                nowp = self.safe_get_robot_pose("kachaka")
                if self._close_pose(nowp, kachaka, xy_tol=0.12, th_tol=0.40):
                    print("[kachaka_move_to_pose] finished")
                    return True
            except Exception:
                pass

            try:
                if not self.kachaka.is_command_running():
                    time.sleep(0.2)
                    nowp2 = self.safe_get_robot_pose("kachaka")
                    if self._close_pose(nowp2, kachaka, xy_tol=0.15, th_tol=0.50):
                        print("[kachaka_move_to_pose] finished")
                        return True
            except Exception:
                pass

            time.sleep(0.05)

        print("[kachaka_move_to_pose] timeout")
        return False

    def kachaka_move_to_xy(self, x, y, theta_for_command=None, timeout_sec=120.0):
        self.spin_and_beacon()
        used_theta = self.oldTheta if theta_for_command is None else float(theta_for_command)

        self.refbox.get_logger().info(f"[kachaka_move_to_xy in the field]: ({x}, {y}, {used_theta})")

        pose = Pose2D()
        pose.x = float(x)
        pose.y = float(y)
        pose.theta = used_theta

        kachaka = self.field2kachaka(pose)

        try:
            now_pose = self.safe_get_robot_pose("kachaka")
            self.refbox.get_logger().info(f"[kachaka_move_to_xy] pose_now: {now_pose}")
            if self._close_xy(now_pose, kachaka, tol=0.20):
                print("[kachaka_move_to_xy] already arrived")
                return True
        except Exception:
            pass

        print(f"[kachaka_move_to_xy in kachaka]: ({kachaka.x}, {kachaka.y}, {kachaka.theta})")
        self.launch_kachaka_move(kachaka)
        print("[kachaka_move_to_xy] wait for move")

        try:
            prev_pose = self.safe_get_robot_pose("kachaka")
        except Exception:
            prev_pose = kachaka

        self.wait_for_motion_start(prev_pose, timeout_sec=8.0)

        done_t = time.time()
        while time.time() - done_t < timeout_sec:
            self.spin_and_beacon()
            try:
                nowp = self.safe_get_robot_pose("kachaka")
                if self._close_xy(nowp, kachaka, tol=0.20):
                    print("[kachaka_move_to_xy] finished by XY")
                    return True
            except Exception:
                pass

            try:
                if not self.kachaka.is_command_running():
                    time.sleep(0.2)
                    nowp2 = self.safe_get_robot_pose("kachaka")
                    if self._close_xy(nowp2, kachaka, tol=0.25):
                        print("[kachaka_move_to_xy] finished by XY after stop")
                        return True
            except Exception:
                pass

            time.sleep(0.05)

        print("[kachaka_move_to_xy] timeout")
        return False

    # ------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------
    def getNextPoint(self, pointNumber):
        route = self.refbox.refboxNavigationRoutes.route
        if pointNumber >= len(route):
            return None

        zone = route[pointNumber].zone
        print("getNextPoint:", zone)
        print("gazebo zone:", zone)
        return self.zoneToXY(zone)

    def move_with_retry(self, field_x, field_y, target_theta, max_retry=3):
        for trial in range(max_retry):
            print(f"[move_with_retry] trial {trial + 1}/{max_retry}")

            try:
                now_field = self.safe_get_robot_pose("field")
                print(f"[move_with_retry] now_field=({now_field.x}, {now_field.y}, {now_field.theta})")
            except Exception as e:
                print(f"[move_with_retry] failed to get current field pose: {e}")
                now_field = None

            # 対角で大きい移動は中継点を挟む
            if now_field is not None:
                dx = abs(float(now_field.x) - float(field_x))
                dy = abs(float(now_field.y) - float(field_y))

                if dx > 1.0 and dy > 1.0:
                    # まずYを合わせてからXを合わせる
                    mid_x = float(now_field.x)
                    mid_y = float(field_y)
                    print(f"[move_with_retry] via midpoint=({mid_x}, {mid_y})")

                    ok_mid = self.kachaka_move_to_xy(mid_x, mid_y, self.oldTheta, timeout_sec=90.0)
                    if not ok_mid:
                        print("[move_with_retry] midpoint move failed")
                        time.sleep(0.5)
                        continue

            ok_xy = self.kachaka_move_to_xy(field_x, field_y, self.oldTheta, timeout_sec=120.0)
            if not ok_xy:
                print("[move_with_retry] final XY move failed")
                time.sleep(0.5)
                continue

            # 向き調整は致命扱いしない
            ok_theta = self.kachaka_move_to_pose(field_x, field_y, target_theta)
            if not ok_theta:
                print("[move_with_retry] theta adjustment failed, but XY arrived -> treat as success")

            return True

        return False

    def navToPoint(self, point):
        if point is None:
            return False

        self.kachaka_speak("Go to Zone " + str(point.x) + " and " + str(point.y))

        field_x, field_y = self.zone_center_to_field_xy(point.x, point.y)
        ok = self.move_with_retry(field_x, field_y, point.theta, max_retry=3)

        self.kachaka_speak("finished")
        return ok

    def navigation(self):
        print("navigation challenge")
        print("====")
        self.oldTheta = math.pi / 2.0

        while (len(self.refbox.refboxNavigationRoutes.route) == 0 or
               len(self.refbox.refboxMachineInfo.machines) == 0):
            self.spin_and_beacon()

        route = list(self.refbox.refboxNavigationRoutes.route)
        print(f"route len: {len(route)}")

        for pointNumber in range(len(route)):
            print(pointNumber)
            self.spin_and_beacon()

            route = list(self.refbox.refboxNavigationRoutes.route)
            print(route)

            if len(route) == 0:
                print("finished")
                return

            point = self.getNextPoint(pointNumber)
            if point is None:
                print("point is None")
                break

            # 次点から向きを作る
            if pointNumber + 1 < len(route):
                next_point = self.zoneToXY(route[pointNumber + 1].zone)
                cur_fx, cur_fy = self.zone_center_to_field_xy(point.x, point.y)
                nxt_fx, nxt_fy = self.zone_center_to_field_xy(next_point.x, next_point.y)
                point.theta = self.direction_to_theta(cur_fx, cur_fy, nxt_fx, nxt_fy)
                self.oldTheta = point.theta
            else:
                point.theta = self.oldTheta

            print("point:", point)

            if self.navToPoint(point):
                print("arrived #", pointNumber + 1, ": point")
            else:
                print("failed to arrive #", pointNumber + 1)
                break

            for _ in range(4):
                self.spin_and_beacon()

        print("navigation finished")

    # ------------------------------------------------------------
    # Challenge selector
    # ------------------------------------------------------------
    def exploration(self):
        print("exploration challenge")

    def production(self):
        print("production challenge")

    def grasping(self):
        print("grasping challenge")

    def main_exploration(self):
        print("main challenge: exploration")

    def main_production(self):
        print("main challenge: production")

    def test(self):
        print("[test]")

    def challenge(self, name):
        challenge_functions = {
            "exploration": self.exploration,
            "production": self.production,
            "grasping": self.grasping,
            "navigation": self.navigation,
            "main_exploration": self.main_exploration,
            "main_production": self.main_production,
            "test": self.test,
        }

        pose = Pose2D()
        pose.x = -1.0 * self.robotNum - 1.5
        pose.y = 0.5
        pose.theta = math.pi / 2.0
        self.kachakaStartPosition = True

        if name == "grasping":
            startX = [-0.5, -4.5, -0.5]
            startY = [0.5, 1.5, 4.5]
            startTheta = [math.pi / 2.0, math.pi / 2.0, math.pi]
            pose.x = startX[self.robotNum - 1]
            pose.y = startY[self.robotNum - 1]
            pose.theta = startTheta[self.robotNum - 1]
            self.kachakaStartPosition = False
        elif name == "main_exploration" or name == "main_production":
            pose.x = 3.5 + self.robotNum

        print("Team Color: ", self.refbox.teamColor)
        if self.refbox.teamColor == 1:
            pose.x = -pose.x
        print(pose.x, pose.y, pose.theta)

        self.refbox.sendBeacon()
        try:
            print(self.kachaka.get_robot_pose())
        except Exception as e:
            print(f"failed get_robot_pose: {e}")

        self.kachaka_speak(name + "を頑張るよ．")

        # 旧コード準拠の最初の前進は、誤差を増やす可能性があるため外す
        # self.kachaka_move_to_pose(pose.x, pose.y + 1.0, pose.theta)

        if name in challenge_functions:
            print(f"[challenge] {name} challenge start.")
            challenge_functions[name]()
        else:
            print(f"[challenge] Unknown challenge: {name}")

    # ------------------------------------------------------------
    # Optional / legacy
    # ------------------------------------------------------------
    def kachaka_ros_odometry(self):
        self.spin_and_beacon()
        odometry = self.kachaka.get_ros_odometry()
        return odometry


def main(args=None):
    rclpy.init(args=args)

    args = sys.argv
    gazeboFlag = True
    robotNum = 1
    challenge_name = "navigation"

    if len(args) >= 2:
        challenge_name = args[1]
        if len(args) >= 3:
            robotNum = int(args[2])

    refbox = btr2_refbox.refbox(
        teamName="Babytigers-R",
        robotNum=robotNum,
        gazeboFlag=False
    )
    refbox.sendBeacon()

    print(refbox)
    rcll2025 = btr2_rcll(
        teamName="Babytigers-R",
        robotNum=robotNum,
        gazeboFlag=gazeboFlag,
        refbox=refbox
    )

    rcll2025.kachaka_speak("こんにちは、btr2_rcll2025.py を実行中です．")

    challengeFlag = True
    oldGamePhase = -1

    while rclpy.ok():
        try:
            rclpy.spin_once(refbox, timeout_sec=0.1)
        except Exception:
            pass

        if refbox.refboxGameStateFlag:
            print("game2025:", challenge_name, refbox.refboxGameState)

            if challengeFlag:
                if oldGamePhase != refbox.refboxGamePhase:
                    print("refboxGamePhase: ", refbox.refboxGamePhase)
                    oldGamePhase = refbox.refboxGamePhase

                # navigation を直接開始
                if challenge_name == "navigation" and refbox.refboxGamePhase in [10, 20, 30]:
                    rcll2025.challenge("navigation")
                    challengeFlag = False

                elif challenge_name == "exploration" and refbox.refboxGamePhase == 20:
                    rcll2025.challenge("exploration")
                    challengeFlag = False

        time.sleep(0.05)

    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()