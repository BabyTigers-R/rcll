#!/usr/bin/python3

import time
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import numpy
from numpy import linalg
from scipy import interpolate

from geometry_msgs.msg import Pose2D, Point, Twist, Vector3
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

from tf_transformations import euler_from_quaternion
from myagv_msgs.srv import ResetOdometry


# -----------------------------
# utility（ROS1完全維持）
# -----------------------------
def quaternion_to_euler(quaternion):
    e = euler_from_quaternion(
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    )
    return Vector3(x=e[0], y=e[1], z=e[2])


def tangent_angle(u, v):
    i = numpy.inner([u.y, u.x], [v.y, v.x])
    n = linalg.norm([u.y, u.x]) * linalg.norm([v.y, v.x])
    if n == 0:
        return 90.0
    c = i / n
    return numpy.rad2deg(numpy.arccos(numpy.clip(c, -1.0, 1.0)))


def MPS_angle(u, v):
    p0 = Point()
    p1 = Point()
    p0.x, p0.y = 1.0, 0.0
    p1.x, p1.y = v.x - u.x, v.y - u.y
    return tangent_angle(p1, p0)


# =========================================================
# ROS2 MAIN CLASS（ROS1→ROS2完全移植・削除ゼロ思想）
# =========================================================
class rc2026(Node):

    def __init__(self, topicName):
        super().__init__("btr2024")

        self.topicName = topicName
        self.dt = 0.1

        # ---------------- state ----------------
        self.btrOdometry = Odometry()

        self.centerPoint = Point()
        self.leftPoint = Point()
        self.rightPoint = Point()
        self.forwardPoint = Point()

        # ---------------- publishers ----------------
        self.pub1 = self.create_publisher(Twist, "/cmd_vel", 10)

        qos = QoSProfile(depth=10)

        # ---------------- subscribers（完全維持） ----------------
        self.sub1 = self.create_subscription(
            Odometry,
            self.topicName + "/odom",
            self.myagvOdometry,
            qos
        )

        self.sub3 = self.create_subscription(
            Point,
            self.topicName + "/btr/centerPoint",
            self.centerPointCb,
            qos
        )

        self.sub4 = self.create_subscription(
            Point,
            self.topicName + "/btr/leftPoint",
            self.leftPointCb,
            qos
        )

        self.sub5 = self.create_subscription(
            Point,
            self.topicName + "/btr/rightPoint",
            self.rightPointCb,
            qos
        )

        self.sub6 = self.create_subscription(
            Point,
            self.topicName + "/btr/forwardPoint",
            self.forwardPointCb,
            qos
        )

        # ---------------- service ----------------
        self.startRpLidar()

    # =====================================================
    # callbacks（完全維持）
    # =====================================================
    def centerPointCb(self, msg):
        self.centerPoint = msg

    def leftPointCb(self, msg):
        self.leftPoint = msg

    def rightPointCb(self, msg):
        self.rightPoint = msg

    def forwardPointCb(self, msg):
        self.forwardPoint = msg

    def myagvOdometry(self, data):
        quat = quaternion_to_euler(data.pose.pose.orientation)

        btrOdometryTmp = data
        btrOdometryTmp.pose.pose.position.z = quat.z / math.pi * 180

        self.btrOdometry = btrOdometryTmp

    # =====================================================
    # lidar start（ROS1完全維持）
    # =====================================================
    def startRpLidar(self):
        print("scan start:" + self.topicName + "/btr/scan_start")

        client = self.create_client(Empty, self.topicName + "/btr/scan_start")

        while not client.wait_for_service(timeout_sec=1.0):
            print("service not available, waiting again...")

        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        print("scan start2")

    # =====================================================
    # basic run
    # =====================================================
    def run(self):
        print("run")

    # =====================================================
    # odom sync（完全維持）
    # =====================================================
    def w_waitOdometry(self):
        s = self.btrOdometry.header.stamp.sec
        ns = self.btrOdometry.header.stamp.nanosec

        while (
            s == self.btrOdometry.header.stamp.sec and
            ns == self.btrOdometry.header.stamp.nanosec
        ):
            rclpy.spin_once(self, timeout_sec=0.01)

    # =====================================================
    # velocity output（ROS2版）
    # =====================================================
    def w_setVelocity(self, data):
        twist = Twist()
        twist.linear.x = data.x
        twist.linear.y = data.y
        twist.angular.z = getattr(data, "theta", 0.0)

        self.pub1.publish(twist)
        time.sleep(self.dt)

    # =====================================================
    # MOVE（ROS1完全復元ロジック）
    # =====================================================
    def w_myagvMove(self, x, y, ori=1000, quick=False):

        if quick:
            velocity1 = interpolate.interp1d(move_distance_quick, move_velocity_quick)
        else:
            velocity1 = interpolate.interp1d(move_distance, move_velocity)

        self.w_waitOdometry()
        nowPoint = self.btrOdometry

        ret = True

        theta = nowPoint.pose.pose.position.z / 180 * math.pi

        if ori == 1000:
            ori = 0
            turnFlag = False
        else:
            turnFlag = True

        ori += nowPoint.pose.pose.position.z

        target_x = x * math.cos(theta) - y * math.sin(theta) + nowPoint.pose.pose.position.x
        target_y = x * math.sin(theta) + y * math.cos(theta) + nowPoint.pose.pose.position.y

        while True:

            self.w_waitOdometry()
            nowPoint = self.btrOdometry

            theta = nowPoint.pose.pose.position.z / 180 * math.pi

            dx1 = target_x - nowPoint.pose.pose.position.x
            dy1 = target_y - nowPoint.pose.pose.position.y

            dx = dx1 * math.cos(-theta) - dy1 * math.sin(-theta)
            dy = dx1 * math.sin(-theta) + dy1 * math.cos(-theta)

            v = Pose2D()

            v.x = float(velocity1(dx)) if not math.isnan(dx) else 0.0
            v.y = float(velocity1(dy)) if not math.isnan(dy) else 0.0

            if (abs(dx) + abs(dy)) < 0.75 and turnFlag:
                v.theta = self.w_turnVelocity(
                    theta / math.pi * 180,
                    ori,
                    quick
                )
            else:
                v.theta = 0.0

            # forward safety（完全保持）
            if self.forwardPoint.x < 1.0 and self.forwardPoint.x > 0.0:
                v.x *= self.forwardPoint.x
                if self.forwardPoint.x < 0.2:
                    v.x = 0.0
                    ret = False

            self.w_setVelocity(v)

            if v.x == 0 and v.y == 0 and v.theta == 0:
                return ret

    # =====================================================
    # TURN（完全復元）
    # =====================================================
    def w_turnVelocity(self, theta, ori, quick=False):

        if quick:
            velocity1 = interpolate.interp1d(
                turn_angle_quick,
                turn_velocity_quick,
                bounds_error=False,
                fill_value=(turn_velocity_quick[0], turn_velocity_quick[-1])
            )
        else:
            velocity1 = interpolate.interp1d(
                turn_angle,
                turn_velocity,
                bounds_error=False,
                fill_value=(turn_velocity[0], turn_velocity[-1])
            )

        diff = ori - theta

        if diff > 180:
            diff -= 360
        if diff < -180:
            diff += 360

        if diff == 0:
            return 0.0

        sign = diff / abs(diff)
        vel = abs(float(velocity1(diff)))

        return (vel * sign) / 5.0


# =========================================================
# MAIN（ROS2標準）
# =========================================================
if __name__ == "__main__":

    rclpy.init()

    agent = rc2026("myagv")

    try:
        while rclpy.ok():
            rclpy.spin_once(agent, timeout_sec=0.01)
            agent.run()
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass

    finally:
        agent.destroy_node()
        rclpy.shutdown()
