import cv2
from cv2 import aruco
import numpy as np
import math
import time
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import os
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Pose, Quaternion, Twist, TwistStamped, TwistWithCovariance
from btr2_msgs.srv import ResetOdometry, GetPose

import kachaka_api

class btr2_odom(Node):
  def __init__(self):
    super().__init__('btr2_odom')
    # QoS の設定: RELIABILITY を RELIABLE に
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    self.sub01 = self.create_subscription(
        Odometry, "/kachaka/odometry/odometry", self.get_odometry, qos_profile)

    self.srv01 = self.create_service(
        ResetOdometry, "/reset_odometry", self.reset_odometry)
    self.srv02 = self.create_service(
        GetPose, "/get_pose", self.get_pose)

    self.pub01 = self.create_publisher(
        Odometry, "/odom", 10)

    self.origin_position = None  # kachaka座標系原点とresetで指定された原点との差
    self.origin_theta = 0.0        # 回転オフセット
    self.latest_kachaka_odom = None
    self.initialized = False

    self.origin_position = (0, 0)
    self.origin_theta = 0
    self.reset_position = (0, 0)
    self.reset_theta = 0

    req = ResetOdometry.Request()
    req.x = 0.0
    req.y = 0.0
    req.phi = 0.0

    res = ResetOdometry.Response()
    self.reset_odometry(req, res)

    #timer_period = 0.1
    #self.timer = self.create_timer(timer_period, self.timer_callback)

    # def timer_callback(self):
    # self.pub01.publish(self.pub01)

    kachakaIP = os.getenv('kachaka_IP')
    self.client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")

  def quaternion_from_euler(self, roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q

  def euler_from_quaternion(self, quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
    # return roll

  def reset_odometry(self, request, response):
    # 現在のkachakaの位置・姿勢を取得し，それをresetの基準として扱う
    if self.latest_kachaka_odom is None:
      self.get_logger().warn("まだodometryが受信されていません")
      #response.success = False
      # return response
      return response

    # 現在の kachaka の位置・角度を origin_pose として保存
    current_pose = self.latest_kachaka_odom # self.latest_kachaka_odom.pose.pose
    self.origin_position = (current_pose.x, current_pose.y) # (current_pose.position.x, current_pose.position.y)
    self.origin_theta = current_pose.theta # (current_pose.orientation)
    print(current_pose)
    # print(f"[reset_odometry] {current_pose.theta}, {current_pose.orientation}")

    # サービスで指定されたリセット後の odom 姿勢
    self.reset_position = (request.x, request.y)
    self.reset_theta = request.phi  # radian

    self.get_logger().info(f"Reset odometry to ({request.x}, {request.y}, {request.phi})")

    # response.success = True
    # return response
    return response

  def get_pose(self, msg, response):
    target_pose = msg

    # 相対変化量
    dx = target_pose.x - self.reset_position[0]
    dy = target_pose.y - self.reset_position[1]

    # 姿勢変化に対する回転補正（kachaka座標系に合わせる）
    cos_theta = math.cos(-self.origin_theta - self.reset_theta)
    sin_theta = math.sin(-self.origin_theta - self.reset_theta)
    x_kachaka = cos_theta * dx - sin_theta * dy
    y_kachaka = sin_theta * dx + cos_theta * dy

    # response = response
    response.x = self.origin_position[0] - x_kachaka
    response.y = self.origin_position[1] - y_kachaka
    response.phi = target_pose.phi - self.reset_theta + self.origin_theta

    print("[get_pose]: ", response)
    return response
    
  def get_odometry(self, msg):
    # print("[get_odometry] msg: ", msg)
    new_odom = Odometry()
    new_odom.twist = msg.twist
    msg = self.client.get_robot_pose()
    # print("[get_robot_pose] msg: ", msg)

    self.latest_kachaka_odom = msg

    if self.origin_position is None:
        return

    # 現在の kachaka 姿勢
    x = msg.x # msg.pose.pose.position.x
    y = msg.y # msg.pose.pose.position.y
    theta = msg.theta # self.euler_from_quaternion(msg.pose.pose.orientation) # radian
    # print(f"[get_odometry] {x}, {y}, {theta}")

    # 相対変化量（リセット時からの差分）
    dx = x - self.origin_position[0]
    dy = y - self.origin_position[1]
    dtheta = theta - self.origin_theta

    # 姿勢変化に対する回転補正（reset座標系に合わせる）
    cos_theta = math.cos(-self.origin_theta - self.reset_theta)
    sin_theta = math.sin(-self.origin_theta - self.reset_theta)
    x_rel = cos_theta * dx - sin_theta * dy
    y_rel = sin_theta * dx + cos_theta * dy

    # /odom = reset位置 + 相対変化量
    x_odom = self.reset_position[0] + x_rel
    y_odom = self.reset_position[1] + y_rel
    theta_odom = self.reset_theta + dtheta

    # /odom 発行
    # new_odom = Odometry()
    new_odom.header.stamp = self.get_clock().now().to_msg()
    new_odom.header.frame_id = "odom"
    new_odom.child_frame_id = "base_footprint" # "msg.child_frame_id
    new_odom.pose.pose.position.x = x_odom
    new_odom.pose.pose.position.y = y_odom
    new_odom.pose.pose.position.z = 0.0 # + theta_odom # zを流用
    new_odom.pose.pose.orientation = self.quaternion_from_euler(0, 0, theta_odom)
    # new_odom.twist = msg.twist
    self.pub01.publish(new_odom)
    # print("[get_odometry] ", new_odom)

    # get_odometry 内の末尾で
    if not self.initialized:
        req = ResetOdometry.Request()
        req.x = 0.0
        req.y = 0.0
        req.phi = 0.0
        res = ResetOdometry.Response()
        self.reset_odometry(req, res)
        self.initialized = True

def main(args=None):
  rclpy.init(args=args)
  btr2 = btr2_odom()
  try:
    rclpy.spin(btr2)
  except KeyboardInterrupt:
    pass
  finally:
    if rclpy.ok():  # ✅ shutdownされていないときだけ呼ぶ
      rclpy.shutdown()

if __name__ == "__main__":
  main()
