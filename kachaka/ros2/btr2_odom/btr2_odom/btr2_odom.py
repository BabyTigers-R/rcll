import math
import time
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Pose, Quaternion, Twist, TwistStamped, TwistWithCovariance

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

    self.pub01 = self.create_publisher(
        Odometry, "/odom", 10)

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

  def get_odometry(self, msg):
    msg = self.client.get_robot_pose() # (x, y, theta)

    # /odom 発行
    new_odom = Odometry()
    new_odom.header.stamp = self.get_clock().now().to_msg()
    new_odom.header.frame_id = "odom"
    new_odom.child_frame_id = "base_footprint" # "msg.child_frame_id
    new_odom.pose.pose.position.x = -msg.y + 2.5 - 5.0 # Magenta: -2.5, Cyan: +2.5
    new_odom.pose.pose.position.y = msg.x + 0.5
    new_odom.pose.pose.position.z = 0.0 # + msg.theta + 3.14159/2.0# zを流用
    new_odom.pose.pose.orientation = self.quaternion_from_euler(0, 0, msg.theta + 3.14159/2.0)
    # new_odom.twist = msg.twist
    self.pub01.publish(new_odom)
    print(f"[get_odometry] ({new_odom.pose.pose.position})")

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
