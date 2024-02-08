#!/usr/bin/env python
#!/usr/bin/python

START_ANGLE = -90 # -90
END_ANGLE = 90 # 90
START_EDGE_ANGLE = -60 # -120
END_EDGE_ANGLE = 60 # -60
THRESHOLD_ANGLE = 5 

import rospy
import math
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse

maxSensorDist = 20
topicName = ""
#
def scanDistanceInf(deg):
  global topicName
  if (topicName == ""):
    return scanData.ranges[int(len(scanData.ranges) / 360 * ((deg + 360 - 90) % 360))]
  else:
    # the range of gazebo's laser is from START_ANGLE to END_ANGLE?
    return scanData.ranges[int(len(scanData.ranges) / (END_ANGLE - START_ANGLE) * (deg - START_ANGLE))]

def scanDistance(deg):
  dist = scanDistanceInf(deg)
  if (math.isinf(dist) or math.isnan(dist)):
      return dist # maxSensorDist
  else:
      return dist
#
def polarToPoint(distance, angle):
  global topicName
  point = Point()
  # if (topicName == ""):
  #   radian = math.radians(angle - 90)
  #   point.z = angle - 90
  # else:
  radian = math.radians(angle)
  point.z = angle
  # point.x = distance * math.cos(radian)
  # point.y = distance * math.sin(radian)
  point.x = distance * math.cos(radian)
  point.y = distance * math.sin(radian)
  # point.z = angle + 0
  return point

#
def calcAngle(pointA, pointB):
  point = Point()
  point.x = pointA.x - pointB.x
  point.y = pointA.y - pointB.y
  return math.degrees(math.atan2(point.y, point.x))

#
def findEdge(startAngle, angleStep):
  startPoint = polarToPoint(scanDistance(startAngle - angleStep), startAngle - angleStep)
  oldPoint = startPoint
  i = startAngle
  oldAngle = -360

  while True:
    nowPoint = polarToPoint(scanDistance(i), i)
    # if (math.isinf(scanDistance(i))):
    #   break
    microAngle = calcAngle(oldPoint, nowPoint)
    macroAngle = calcAngle(startPoint, nowPoint)
    diff = abs(microAngle - macroAngle)
    if (diff > 15):
      # print("findEdge", startPoint, nowPoint, diff)
      break
    i = i + angleStep
    if (i < -180 or i > 180):
      break
    oldPoint = nowPoint
  
  # print("findEdge: ", i - angleStep, scanDistance(i - angleStep))
  return polarToPoint(scanDistance(i - angleStep), i - angleStep)

#
def calcPoint():
  global centerPoint, closePoint, leftPoint, rightPoint, forwardPoint
  CENTER_ANGLE = (START_ANGLE + END_ANGLE) / 2
  minDistance = scanDistance(CENTER_ANGLE)
  minAngle = CENTER_ANGLE
  centerPoint = polarToPoint(minDistance, minAngle)
  leftPoint5 = polarToPoint(scanDistance(CENTER_ANGLE + 5), CENTER_ANGLE + 5)
  rightPoint5 = polarToPoint(scanDistance(CENTER_ANGLE - 5), CENTER_ANGLE - 5)
  centerPoint.z = calcAngle(leftPoint5, rightPoint5)
  # print(minDistance, minAngle)
  # print(len(scanData.ranges) / 360 , (((minAngle + 180 + 45) + 360) % 360))

  for i in range(START_EDGE_ANGLE, END_EDGE_ANGLE):
    if (minDistance > scanDistance(i)):
      minDistance = scanDistance(i)
      minAngle = i
  # print("minAngle:", minAngle, ", minDistance:", minDistance)
  closePoint = polarToPoint(minDistance, minAngle)

  # find the left edge and right edge
  leftPoint  = findEdge(minAngle - 1, +1)
  rightPoint = findEdge(minAngle + 1, -1)
  # print("centerAng:", minAngle, "left:", leftPoint.z, "right:", rightPoint.z)
  # print("dist", ((leftPoint.x - rightPoint.x) ** 2 + (leftPoint.y - rightPoint.y) **2) ** 0.5)

  forwardPoint = polarToPoint(scanDistance(START_ANGLE + END_ANGLE) / 2, (START_ANGLE + END_ANGLE) / 2)
  radius = 0.24
  for i in range(START_EDGE_ANGLE, END_EDGE_ANGLE):
    obstaclePoint = polarToPoint(scanDistance(i), i)
    if (forwardPoint.x > obstaclePoint.x):
      if (-radius < obstaclePoint.y and obstaclePoint.y < radius):
        forwardPoint = obstaclePoint


#
def laserScan(data):
  global scanData
  scanData = data
  # scanNumber = len(scanData.ranges)
  if (scanFlag == True):
    calcPoint()
  # else:
  #   print   "0:", scanDistance(  0), \
  #          "90:", scanDistance( 90), \
  #         "180:", scanDistance(180), \
  #         "270:", scanDistance(-90)

#
def btrScanStart(self):
  global scanFlag
  scanFlag = True
  print("start publishing")
  return EmptyResponse()
#
def btrScanStop(self):
  global scanFlag
  scanFlag = False
  print("stop publishing")
  return EmptyResponse()

# main
#
if __name__ == '__main__':
  args = sys.argv
  topicName = ""
  nodeName = "btr_scan"
  if (len(args) >= 2):
    if ( args[1] == "gazebo" or args[1] == "-g" or args[1] == "--gazebo"):
      topicName = "/robotino" + str(args[2]) 
      nodeName = "robotino_scan" + str(args[2])
  scanFlag = False
  centerPoint = Point()
  closePoint = Point()
  leftPoint = Point()
  rightPoint = Point()
  forwardPoint = Point()

  rospy.init_node(nodeName)
  sub01 = rospy.Subscriber(topicName + "/scan", LaserScan, laserScan)
  srv01 = rospy.Service(topicName + "/btr/scan_start", Empty, btrScanStart)
  srv02 = rospy.Service(topicName + "/btr/scan_stop", Empty, btrScanStop)
  pub00 = rospy.Publisher(topicName + "/btr/centerPoint", Point, queue_size = 10)
  pub01 = rospy.Publisher(topicName + "/btr/closePoint", Point, queue_size = 10)
  pub02 = rospy.Publisher(topicName + "/btr/leftPoint", Point, queue_size = 10)
  pub03 = rospy.Publisher(topicName + "/btr/rightPoint", Point, queue_size = 10)
  pub04 = rospy.Publisher(topicName + "/btr/forwardPoint", Point, queue_size = 10)
  rate = rospy.Rate(10)

  # rospy.spin()

  scanData = LaserScan
  
  while not rospy.is_shutdown():
    if (scanFlag == True):
      pub00.publish(centerPoint)
      pub01.publish(closePoint)
      pub02.publish(leftPoint)
      pub03.publish(rightPoint)
      pub04.publish(forwardPoint)
    rate.sleep()

