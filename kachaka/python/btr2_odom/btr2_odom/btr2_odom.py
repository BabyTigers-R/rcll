import cv2
from cv2 import aruco
import numpy as np
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import os
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Pose, Quaternion 
from rto_msgs.srv import ResetOdometry

def euler_from_quaternion(quaternion):
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

def quaternion_from_euler(roll, pitch, yaw):
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

class btr2_odom(Node):
  def __init__(self):
    super().__init__('btr2_odom')
    # QoS の設定: RELIABILITY を RELIABLE に
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )
    self.sub01 = self.create_subscription(Odometry, "/kachaka/odometry/odometry", self.getOdometry, qos_profile)
    self.srv01 = self.create_service(ResetOdometry, "/reset_odometry",  self.resetOdometry)
    self.pub01 = self.create_publisher(Odometry, '/odom', 10)

    self.odom = Odometry()
    self.resetOdomOrigin = Odometry()
    self.resetOdomPoint  = Odometry()

    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    self.pub01.publish(self.odom)

  def resetOdometry(self, request, response):
    orientation = request.pose.pose.orientation
    print(orientation)

    self.resetOdomOrigin = request.ResetOdometry
    self.resetOdomPoint = self.nowOdom
    return response

  def getOdometry(self, data):
    print(data.pose.pose)
    self.nowOdom = data
    # calc distance from reset point to current point.
    distance_x = data.pose.pose.position.x - self.resetOdomPoint.pose.pose.position.x
    distance_y = data.pose.pose.position.y - self.resetOdomPoint.pose.pose.position.y
    distance_length = (distance_x ** 2 + distance_y ** 2) ** 0.5

    # calc angle
    angle_odom = np.arctan2(distance_y, distance_x)
    angle_now = (euler_from_quaternion(self.resetOdomPoint.pose.pose.orientation) - euler_from_quaternion(self.resetOdomOrigin.pose.pose.orientation))
    angle_point = angle_odom - angle_now

    self.odom.pose.pose.position.x = distance_length * math.cos(angle_point)
    self.odom.pose.pose.position.y = distance_length * math.sin(angle_point)
    self.odom.pose.pose.orientation = quaternion_from_euler(angle_point, 0, 0)
    self.odom = self.nowOdom

def main(args=None):
  rclpy.init(args=args)
  btr2 = btr2_odom()
  rclpy.spin(btr2)
  rclpy.shutdown()

if __name__ == "__main__":
  main()
