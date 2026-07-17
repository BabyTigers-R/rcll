#!/usr/bin/env python3

START_ANGLE = -90 # -90
END_ANGLE = 90 # 90
START_EDGE_ANGLE = -60 # -120
END_EDGE_ANGLE = 60 # -60
THRESHOLD_ANGLE = 5 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
import sys
import time
import RPi.GPIO as GPIO
 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_srvs.srv import Empty

maxSensorDist = 20
topicName = ""

# --- GPIOピンの定義 ---
LIDAR_CTRL_PIN = 18  

class BtrScanNode(Node):
    def __init__(self, node_name, topic_name):
        super().__init__(node_name)
        global topicName
        topicName = topic_name
        
        self.scanFlag = False
        self.scanData = None
        
        self.centerPoint = Point()
        self.closePoint = Point()
        self.leftPoint = Point()
        self.rightPoint = Point()
        self.forwardPoint = Point()

        # --- Raspberry Pi GPIO 初期設定 ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LIDAR_CTRL_PIN, GPIO.OUT)
        GPIO.output(LIDAR_CTRL_PIN, GPIO.LOW) # 初期状態はLOW

        qos = QoSProfile(depth=10)

        # パブリッシャーとサブスクライバーの設定
        self.sub01 = self.create_subscription(LaserScan, topicName + "/scan", self.laserScan, qos)
        
        self.pub00 = self.create_publisher(Point, topicName + "/btr/centerPoint", qos)
        self.pub01 = self.create_publisher(Point, topicName + "/btr/closePoint", qos)
        self.pub02 = self.create_publisher(Point, topicName + "/btr/leftPoint", qos)
        self.pub03 = self.create_publisher(Point, topicName + "/btr/rightPoint", qos)
        self.pub04 = self.create_publisher(Point, topicName + "/btr/forwardPoint", qos)

        # サービスサーバーの設定
        self.srv01 = self.create_service(Empty, topicName + "/btr/scan_start", self.btrScanStart)
        self.srv02 = self.create_service(Empty, topicName + "/btr/scan_stop", self.btrScanStop)

        # パブリッシュ用のタイマー（10Hz = 0.1秒周期）
        self.timer = self.create_timer(0.1, self.publish_topics)

    def laserScan(self, data):
        self.scanData = data
        if self.scanFlag:
            self.calcPoint()

    def btrScanStart(self, request, response):
        self.scanFlag = True
        self.get_logger().info("start publishing & GPIO HIGH")
        
        # GPIOをHIGHにする
        
        GPIO.output(LIDAR_CTRL_PIN, GPIO.HIGH)
        
        return response

    def btrScanStop(self, request, response):
        self.scanFlag = False
        self.get_logger().info("stop publishing & GPIO LOW")
        
        # GPIOをLOWにする
        GPIO.output(LIDAR_CTRL_PIN, GPIO.LOW)
        
        return response

    def publish_topics(self):
        if self.scanFlag:
            self.pub00.publish(self.centerPoint)
            self.pub01.publish(self.closePoint)
            self.pub02.publish(self.leftPoint)
            self.pub03.publish(self.rightPoint)
            self.pub04.publish(self.forwardPoint)

    # 計算ロジックは元のグローバル参照を維持するため、内部でscanData等を同期
    def calcPoint(self):
        global scanData, centerPoint, closePoint, leftPoint, rightPoint, forwardPoint
        scanData = self.scanData
        
        # 元のロジックを実行
        CENTER_ANGLE = (START_ANGLE + END_ANGLE) / 2
        minDistance = scanDistance(CENTER_ANGLE)
        minAngle = CENTER_ANGLE
        self.centerPoint = polarToPoint(minDistance, minAngle)
        leftPoint5 = polarToPoint(scanDistance(CENTER_ANGLE + 5), CENTER_ANGLE + 5)
        rightPoint5 = polarToPoint(scanDistance(CENTER_ANGLE - 5), CENTER_ANGLE - 5)
        self.centerPoint.z = calcAngle(leftPoint5, rightPoint5)

        for i in range(START_EDGE_ANGLE, END_EDGE_ANGLE):
            if minDistance > scanDistance(i):
                minDistance = scanDistance(i)
                minAngle = i
        self.closePoint = polarToPoint(minDistance, minAngle)

        self.leftPoint  = findEdge(minAngle - 1, +1)
        self.rightPoint = findEdge(minAngle + 1, -1)

        self.forwardPoint = self.centerPoint 
        radius = 0.23
        minRange = 0.08 * 1.1 
        for i in range(START_EDGE_ANGLE, END_EDGE_ANGLE):
            obstaclePoint = polarToPoint(scanDistance(i), i)
            if self.forwardPoint.x > obstaclePoint.x and obstaclePoint.x > minRange:
                if -radius < obstaclePoint.y and obstaclePoint.y < radius:
                    self.forwardPoint = obstaclePoint


# --- 元のアルゴリズム関数（そのままROS 2環境で呼べるよう維持） ---
def scanDistanceInf(deg):
  global topicName, scanData
  if (topicName == ""):
    return scanData.ranges[int(len(scanData.ranges) / 360 * ((deg + 360 - 90) % 360))]
  if (len(scanData.ranges) <= len(scanData.ranges) / (END_ANGLE - START_ANGLE) * (deg - START_ANGLE)):
    return math.inf
  return scanData.ranges[int(len(scanData.ranges) / (END_ANGLE - START_ANGLE) * (deg - START_ANGLE))]

def scanDistance(deg):
  distCenter   = scanDistanceInf(deg)
  distAverage  = (scanDistanceInf(deg - 0.5) + scanDistanceInf(deg + 0.5)) / 2
  distDiff     = abs(distCenter - distAverage)
  distDiffRate = distDiff / distCenter
  if (math.isinf(distCenter) or math.isnan(distCenter)):
      return distCenter
  if (distDiffRate < 0.1):
      return distCenter
  return math.nan

def polarToPoint(distance, angle):
  point = Point()
  radian = math.radians(angle)
  point.z = angle
  point.x = distance * math.cos(radian)
  point.y = distance * math.sin(radian)
  return point

def calcAngle(pointA, pointB):
  point = Point()
  point.x = pointA.x - pointB.x
  point.y = pointA.y - pointB.y
  return math.degrees(math.atan2(point.y, point.x))

def findEdge(startAngle, angleStep):
  startPoint = polarToPoint(scanDistance(startAngle - angleStep), startAngle - angleStep)
  oldPoint = startPoint
  i = startAngle
  while True:
    nowPoint = polarToPoint(scanDistance(i), i)
    microAngle = calcAngle(oldPoint, nowPoint)
    macroAngle = calcAngle(startPoint, nowPoint)
    diff = abs(microAngle - macroAngle)
    if (diff > 15):
      break
    i = i + angleStep
    if (i < -START_ANGLE or i > END_ANGLE):
      break
    oldPoint = nowPoint
  return polarToPoint(scanDistance(i - angleStep), i - angleStep)


# main
if __name__ == '__main__':
  rclpy.init()
  
  args = sys.argv
  t_name = ""
  n_name = "btr_scan"
  if (len(args) >= 2):
    if ( args[1] == "gazebo" or args[1] == "-g" or args[1] == "--gazebo"):
      t_name = "/robotino" + str(args[2]) 
      n_name = "robotino_scan" + str(args[2])

  node = BtrScanNode(n_name, t_name)
 
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()