import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

START_ANGLE = -90 # -90
END_ANGLE = 90 # 90
START_EDGE_ANGLE = -60 # -120
END_EDGE_ANGLE = 60 # -60
THRESHOLD_ANGLE = 5

class btr2_lrf(Node):
  def __init__(self):
    super().__init__('btr2_lrf')
    # QoS の設定: RELIABILITY を RELIABLE に
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    self.topicName = ""
    self.sub01 = self.create_subscription(
        LaserScan, "/kachaka/lidar/scan", self.laser_scan, qos_profile)

    self.pub01 = self.create_publisher(Point, self.topicName + "/btr/centerPoint", 10)
    self.pub02 = self.create_publisher(Point, self.topicName + "/btr/closePoint", 10)
    self.pub03 = self.create_publisher(Point, self.topicName + "/btr/leftPoint", 10)
    self.pub04 = self.create_publisher(Point, self.topicName + "/btr/rightPoint", 10)
    self.pub05 = self.create_publisher(Point, self.topicName + "/btr/forwardPoint", 10)

    # kachakaIP = os.getenv('kachaka_IP')
    # self.client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")

  def laser_scan(self, data):
    self.scanData = data
    self.calcPoint()
    print(f"[centerPoint] {self.centerPoint}")
    self.pub01.publish(self.centerPoint)
    # print(f"[closePoint] {self.closePoint}")
    self.pub02.publish(self.closePoint)
    # print(f"[leftPoint] {self.leftPoint}")
    self.pub03.publish(self.leftPoint)
    # print(f"[rightPoint] {self.rightPoint}")
    self.pub04.publish(self.rightPoint)
    # print(f"[forwardPoint] {self.forwardPoint}")
    self.pub05.publish(self.forwardPoint)

  def calcPoint(self):
    CENTER_ANGLE = (START_ANGLE + END_ANGLE) / 2
    minDistance = self.scanDistance(CENTER_ANGLE)
    minAngle = CENTER_ANGLE
    self.centerPoint = self.polarToPoint(minDistance, minAngle)
    leftPoint5 = self.polarToPoint(self.scanDistance(CENTER_ANGLE + 5), CENTER_ANGLE + 5)
    rightPoint5 = self.polarToPoint(self.scanDistance(CENTER_ANGLE - 5), CENTER_ANGLE - 5)
    self.centerPoint.z = self.calcAngle(leftPoint5, rightPoint5)

    for i in range(START_EDGE_ANGLE, END_EDGE_ANGLE):
      if (minDistance > self.scanDistance(i)):
        minDistance = self.scanDistance(i)
        minAngle = i
    self.closePoint = self.polarToPoint(minDistance, minAngle)

    # find the left edge and right edge
    self.leftPoint  = self.findEdge(minAngle - 1, +1)
    self.rightPoint = self.findEdge(minAngle + 1, -1)

    self.forwardPoint = self.centerPoint
    radius = 0.23
    minRange = 0.08 * 1.1 # consider gausian noize.
    for i in range(START_EDGE_ANGLE, END_EDGE_ANGLE):
      obstaclePoint = self.polarToPoint(self.scanDistance(i), i)
      if (self.forwardPoint.x > obstaclePoint.x and obstaclePoint.x > minRange):
        if (-radius < obstaclePoint.y and obstaclePoint.y < radius):
          self.forwardPoint = obstaclePoint

  def findEdge(self, startAngle, angleStep):
    startPoint = self.polarToPoint(self.scanDistance(startAngle - angleStep), startAngle - angleStep)
    oldPoint = startPoint
    i = startAngle

    while True:
      nowPoint = self.polarToPoint(self.scanDistance(i), i)
      microAngle = self.calcAngle(oldPoint, nowPoint)
      macroAngle = self.calcAngle(startPoint, nowPoint)
      diff = abs(microAngle - macroAngle)
      if (diff > 15):
        break
      i = i + angleStep
      if (i < -START_ANGLE or i > END_ANGLE):
        break
      oldPoint = nowPoint

    return self.polarToPoint(self.scanDistance(i - angleStep), i - angleStep)

  def polarToPoint(self, distance, angle):
    point = Point()
    radian = math.radians(angle)
    point.x = distance * math.cos(radian)
    point.y = distance * math.sin(radian)
    point.z = float(angle)
    return point

  def calcAngle(self, pointA, pointB):
    point = Point()
    point.x = pointA.x - pointB.x
    point.y = pointA.y - pointB.y
    return math.degrees(math.atan2(point.y, point.x))

  def scanDistanceInf(self, deg):
    # angle_min=-3.1415927410125732, angle_max=3.1415927410125732, angle_increment=0.010117851197719574,
    angle_min = self.scanData.angle_min
    angle_max = self.scanData.angle_max
    angle_inc = self.scanData.angle_increment

    if (self.topicName == ""):
      return float(self.scanData.ranges[int(len(self.scanData.ranges) / 360 * ((deg + 360 - 90 + 180) % 360))])
    return self.scanData.ranges[int(len(self.scanData.ranges) / (END_ANGLE - START_ANGLE) * (deg - START_ANGLE))]

  def scanDistance(self, deg):
    distCenter   = self.scanDistanceInf(deg)
    distAverage  = (self.scanDistanceInf(deg - 0.5) + self.scanDistanceInf(deg + 0.5)) / 2
    distDiff     = abs(distCenter - distAverage)
    # print(distCenter, distAverage, distDiff)
    if (distCenter == 0):
        return math.nan
    else:
        distDiffRate = distDiff / distCenter
    
    if (math.isinf(distCenter) or math.isnan(distCenter)):
      return distCenter # maxSensorDist
    if (distDiffRate < 0.1):
      return distCenter
    return math.nan

def main(args=None):
  rclpy.init(args=args)
  btr2 = btr2_lrf()
  try:
    rclpy.spin(btr2)
  except KeyboardInterrupt:
    pass
  finally:
    if rclpy.ok():  # ✅ shutdownされていないときだけ呼ぶ
      rclpy.shutdown()

if __name__ == "__main__":
  main()
