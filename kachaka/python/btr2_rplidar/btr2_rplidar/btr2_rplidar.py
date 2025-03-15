#!/usr/bin/env python

### setting for RpLidar
# START_ANGLE = -90
# END_ANGLE = 90
# START_EDGE_ANGLE = -60
# END_EDGE_ANGLE = 60
# THRESHOLD_ANGLE = 5 

# setting for Kachaka
# angle_min: -3.1415927410125732
# angle_max: 3.1415927410125732
# angle_increment: 0.01640518382191658
START_ANGLE = -180
END_ANGLE = 180
START_EDGE_ANGLE = -60
END_EDGE_ANGLE = 60
THRESHOLD_ANGLE = 5

# import rospy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class btr2_rplidar(Node):
  def __init__(self):
    # self.twist_pub = self.create_publisher(Twist, '/kachaka/manual_control/cmd_vel', 10)
    # timer_period = 1
    # self.timer = self.create_timer(timer_period, self.timer_callback)
    self.topicName = ""
    self.nodeName = "btr2_scan"
    # if (len(args) >= 2):
    #   if ( args[1] == "gazebo" or args[1] == "-g" or args[1] == "--gazebo"):
    #     self.topicName = "/kachaka" + str(args[2])
    #     self.nodeName = "kachaka_scan" + str(args[2])
    self.topicName = "/kachaka/lidar/scan"
    self.nodeName = "kachaka_scan"
    self.scanFlag = False
    self.centerPoint = Point()
    self.closePoint = Point()
    self.leftPoint = Point()
    self.rightPoint = Point()
    self.forwardPoint = Point()

    self.maxSensorDist = 20
    self.topicName = ""

    # QoS の設定: RELIABILITY を RELIABLE に
    qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

    super().__init__('btr2_rplidar')
    # self.sub01 = self.create_subscription(LaserScan, self.topicName + "/scan", self.laserScan, 10)
    self.sub01 = self.create_subscription(LaserScan, "/kachaka/lidar/scan", self.laserScan, qos_profile)

    self.srv01 = self.create_service(Empty, self.topicName + "/btr/scan_start", self.btrScanStart)
    self.srv02 = self.create_service(Empty, self.topicName + "/btr/scan_stop",  self.btrScanStop)
    self.pub00 = self.create_publisher(Point, self.topicName + "/btr/centerPoint", 10)
    self.pub01 = self.create_publisher(Point, self.topicName + "/btr/closePoint", 10)
    self.pub02 = self.create_publisher(Point, self.topicName + "/btr/leftPoint", 10)
    self.pub03 = self.create_publisher(Point, self.topicName + "/btr/rightPoint", 10)
    self.pub04 = self.create_publisher(Point, self.topicName + "/btr/forwardPoint", 10)

    self.scanData = LaserScan

    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    if (self.scanFlag == True):
      self.pub00.publish(self.centerPoint)
      self.pub01.publish(self.closePoint)
      self.pub02.publish(self.leftPoint)
      self.pub03.publish(self.rightPoint)
      self.pub04.publish(self.forwardPoint)

  def scanDistanceInf(self, deg):
    if (self.topicName == ""):
      return self.scanData.ranges[int(len(self.scanData.ranges) / 360 * ((deg + 360 - 90) % 360))]
    # the range of gazebo's laser is from START_ANGLE to END_ANGLE?
    # number of data is 720 for HOKUYO LRF on gazebo.
    if (len(self.scanData.ranges) <= len(self.scanData.ranges) / (END_ANGLE - START_ANGLE) * (deg - START_ANGLE)):
      print(len(self.scanData.ranges), end = ",")
      print(deg, end = ":")
      print(int(len(self.scanData.ranges) / (END_ANGLE - START_ANGLE) * (deg - START_ANGLE)))
      return math.inf
    return self.scanData.ranges[int(len(self.scanData.ranges) / (END_ANGLE - START_ANGLE) * (deg - START_ANGLE))]

  def scanDistance(self, deg):
    distCenter   = self.scanDistanceInf(deg)
    distAverage  = (self.scanDistanceInf(deg - 0.5) + self.scanDistanceInf(deg + 0.5)) / 2
    distDiff     = abs(distCenter - distAverage)
    if distCenter == 0:
      return distCenter
    distDiffRate = distDiff / distCenter
    if (math.isinf(distCenter) or math.isnan(distCenter)):
      return distCenter # maxSensorDist
    if (distDiffRate < 0.1):
      return distCenter
    return math.nan

  def polarToPoint(self, distance, angle):
    point = Point()
    # if (topicName == ""):
    #   radian = math.radians(angle - 90)
    #   point.z = angle - 90
    # else:
    radian = math.radians(angle)
    point.z = float(angle)
    # point.x = distance * math.cos(radian)
    # point.y = distance * math.sin(radian)
    point.x = distance * math.cos(radian)
    point.y = distance * math.sin(radian)
    # point.z = angle + 0
    return point

  def calcAngle(self, pointA, pointB):
    point = Point()
    point.x = pointA.x - pointB.x
    point.y = pointA.y - pointB.y
    return math.degrees(math.atan2(point.y, point.x))

  def findEdge(self, startAngle, angleStep):
    startPoint = self.polarToPoint(self.scanDistance(startAngle - angleStep), startAngle - angleStep)
    oldPoint = startPoint
    i = startAngle

    while True:
      nowPoint = self.polarToPoint(self.scanDistance(i), i)
      # if (math.isinf(scanDistance(i))):
      #   break
      microAngle = self.calcAngle(oldPoint, nowPoint)
      macroAngle = self.calcAngle(startPoint, nowPoint)
      diff = abs(microAngle - macroAngle)
      if (diff > 15):
        # print("findEdge", startPoint, nowPoint, diff)
        break
      i = i + angleStep
      if (i < -START_ANGLE or i > END_ANGLE):
        break
      oldPoint = nowPoint
   
    # print("findEdge: ", i - angleStep, scanDistance(i - angleStep))
    return self.polarToPoint(self.scanDistance(i - angleStep), i - angleStep)

  def calcPoint(self):
    # global centerPoint, closePoint, leftPoint, rightPoint, forwardPoint
    CENTER_ANGLE = (START_ANGLE + END_ANGLE) / 2
    minDistance = self.scanDistance(CENTER_ANGLE)
    minAngle = CENTER_ANGLE
    self.centerPoint = self.polarToPoint(minDistance, minAngle)
    self.leftPoint5 = self.polarToPoint(self.scanDistance(CENTER_ANGLE + 5), CENTER_ANGLE + 5)
    self.rightPoint5 = self.polarToPoint(self.scanDistance(CENTER_ANGLE - 5), CENTER_ANGLE - 5)
    self.centerPoint.z = self.calcAngle(self.leftPoint5, self.rightPoint5)
    # print(minDistance, minAngle)
    # print(len(self.scanData.ranges) / 360 , (((minAngle + 180 + 45) + 360) % 360))

    for i in range(START_EDGE_ANGLE, END_EDGE_ANGLE):
      if (minDistance > self.scanDistance(i)):
        minDistance = self.scanDistance(i)
        minAngle = i
    # print("minAngle:", minAngle, ", minDistance:", minDistance)
    self.closePoint = self.polarToPoint(minDistance, minAngle)

    # find the left edge and right edge
    self.leftPoint  = self.findEdge(minAngle - 1, +1)
    self.rightPoint = self.findEdge(minAngle + 1, -1)
    # print("centerAng:", minAngle, "left:", leftPoint.z, "right:", rightPoint.z)
    # print("dist", ((leftPoint.x - rightPoint.x) ** 2 + (leftPoint.y - rightPoint.y) **2) ** 0.5)

    # forwardPoint = polarToPoint(scanDistance(START_ANGLE + END_ANGLE) / 2, (START_ANGLE + END_ANGLE) / 2)
    self.forwardPoint = self.centerPoint 
    radius = 0.23
    minRange = 0.08 * 1.1 # consider gausian noize.
    for i in range(START_EDGE_ANGLE, END_EDGE_ANGLE):
      obstaclePoint = self.polarToPoint(self.scanDistance(i), i)
      if (self.forwardPoint.x > obstaclePoint.x and obstaclePoint.x > minRange):
        if (-radius < obstaclePoint.y and obstaclePoint.y < radius):
          self.forwardPoint = obstaclePoint

  def laserScan(self, data):
    # global scanData
    self.scanData = data
    # scanNumber = len(scanData.ranges)
    if (self.scanFlag == True):
      self.calcPoint()
    # else:
    #   print   "0:", scanDistance(  0), \
    #          "90:", scanDistance( 90), \
    #         "180:", scanDistance(180), \
    #         "270:", scanDistance(-90)

  def btrScanStart(self, request, response):
    # global scanFlag
    self.scanFlag = True
    print("start publishing")
    return response

  def btrScanStop(self, request, response):
    # global scanFlag
    self.scanFlag = False
    print("stop publishing")
    return response

def main(args=None):
  rclpy.init(args=args)
  rplidar = btr2_rplidar()
  rclpy.spin(rplidar)
  rclpy.shutdown()

# main
#
if __name__ == '__main__':
  main()
