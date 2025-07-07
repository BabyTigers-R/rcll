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
from btr2_msgs.srv import ResetOdometry

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

    self.pub01 = self.create_publisher(
        Odometry, "/odom", 10)

    self.origin_position = None  # kachaka座標系原点とresetで指定された原点との差
    self.origin_yaw = 0.0        # 回転オフセット
    self.latest_kachaka_odom = None

    self.origin_position = (0, 0)
    self.origin_yaw = 0
    self.reset_position = (0, 0)
    self.reset_yaw = 0

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

    # return roll, pitch, yaw
    return roll

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
    self.origin_yaw = self.euler_from_quaternion(current_pose.theta) # (current_pose.orientation)

    # サービスで指定されたリセット後の odom 姿勢
    self.reset_position = (request.x, request.y)
    self.reset_yaw = request.phi

    self.get_logger().info(f"Reset odometry to ({request.x}, {request.y}, {request.phi})")

    # response.success = True
    # return response
    return response

  def get_odometry(self, msg):

    # print("[get_odometryi] msg: ", msg)
    msg = self.client.get_robot_pose()
    # print("[get_robot_pose] msg: ", msg)

    self.latest_kachaka_odom = msg

    if self.origin_position is None:
        return

    # 現在の kachaka 姿勢
    x = msg.x # msg.pose.pose.position.x
    y = msg.y # msg.pose.pose.position.y
    yaw = msg.theta # self.euler_from_quaternion(msg.pose.pose.orientation)

    # 相対変化量（リセット時からの差分）
    dx = x - self.origin_position[0]
    dy = y - self.origin_position[1]
    dyaw = yaw - self.origin_yaw

    # 姿勢変化に対する回転補正（reset座標系に合わせる）
    cos_theta = math.cos(-self.origin_yaw)
    sin_theta = math.sin(-self.origin_yaw)
    x_rel = cos_theta * dx - sin_theta * dy
    y_rel = sin_theta * dx + cos_theta * dy

    # /odom = reset位置 + 相対変化量
    x_odom = self.reset_position[0] + x_rel
    y_odom = self.reset_position[1] + y_rel
    yaw_odom = self.reset_yaw + dyaw

    # /odom 発行
    new_odom = Odometry()
    new_odom.header.stamp = self.get_clock().now().to_msg()
    new_odom.header.frame_id = "odom"
    new_odom.child_frame_id = "base_footprint" # "msg.child_frame_id
    new_odom.pose.pose.position.x = x_odom
    new_odom.pose.pose.position.y = y_odom
    new_odom.pose.pose.position.z = 0.0
    new_odom.pose.pose.orientation = self.quaternion_from_euler(yaw_odom, 0, 0)
    new_odom.twist = Twist() # msg.twist
    new_odom.twist = TwistWithCovariance(twist=Twist(linear=Vector3(x=-5.634551161237598e-05, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=8.407050101913937e-19)), covariance=([ 8.24115722e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  2.07084051e+10,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  4.98729473e-07,  6.63548867e-42,
        4.61546502e-28,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        6.63548867e-42,  4.94993629e-07, -1.37960660e-49,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  4.61546502e-28, -1.86384904e-49,
        4.94993629e-07,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  9.99989784e-10]))

    self.pub01.publish(new_odom)

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
