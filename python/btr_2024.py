#!/usr/bin/python

import struct
import time
import math
import sys
import rospy
import numpy
from numpy import linalg
from scipy import interpolate
import quaternion
import tf
from geometry_msgs.msg import Vector3


from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point, Quaternion, Twist
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, Int16, UInt32, String, \
                         Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse, Empty, EmptyResponse
from nav_msgs.msg import Odometry
import rcll_ros_msgs
# import rcll_btr_msgs
from rcll_btr_msgs.msg import TagInfoResponse, TagLocationResponse
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity, \
                              SetDistance, TagInfo,     TagLocation
from robotino_msgs.srv import ResetOdometry
from rcll_ros_msgs.msg import MachineReportEntryBTR

from topic_tools.srv import MuxDelete

machineName = { 101 : "C-CS1-O", 102 : "C-CS1-I", 103 : "C-CS2-O", 104 : "C-CS2-I",
                201 : "M-CS1-O", 202 : "M-CS1-I", 203 : "M-CS2-O", 204 : "M-CS2-I",
                111 : "C-RS1-O", 112 : "C-RS1-I", 113 : "C-RS2-O", 114 : "C-RS2-I",
                211 : "M-RS1-O", 212 : "M-RS1-I", 213 : "M-RS2-O", 214 : "M-RS2-I",
                121 : "C-BS-O",  122 : "C-BS-I",  221 : "M-BS-O",  222 : "M-BS-I",
                131 : "C-DS-O",  132 : "C-DS-I",  231 : "M-DS-O",  232 : "M-DS-I",
                141 : "C-SS-O",  142 : "C-SS-I",  241 : "M-SS-O",  242 : "M-SS-I",
                301 : "UMPS-1",  302 : "UMPS-2" }

# linear max velocity is 0.1[m/s] and min is 0.01.
# angular max velocity is 1.0[rad/s?] and min is 0.01
min_mps_distance = 0.6
camera_offset = 0.1
#
# speed is unit/s. and interrupt is 0.1s.
# So, the best speed is diff / 0.1
# The unit of turn angle is Deg, but the unit of turn velocity is Rad.
# So, the best speed is diff / 0.1 / 180 * 3.14 = diff * 0.17 (=0.15).
#setting for Gazebo
# turn_angle    = numpy.array([-999, -25, -15,  -10,  -5,  -0.5, -0.5, 0.5,   0.5,    5,    10,   15,   25, 999])
# turn_velocity = numpy.array([   1, 1.0, 1.0, 0.75, 0.3,  0.07,     0,  0, -0.07, -0.3, -0.75, -1.0, -1.0,  -1])
#
# setting for Real Robot
turn_angle    = numpy.array([-999, -25,  -15,  -10,   -5,  -0.1, -0.1, 0.1,   0.1,     5,   10,   15,   25, 999])
turn_velocity = numpy.array([   2, 2.0,  2.0,  1.5, 0.75,  0.02,    0,   0, -0.02, -0.75, -1.5, -2.0, -2.0,  -2])

# setting for Gazebo
# go_distance = numpy.array([-9999, -0.05, -0.02, -0.015, -0.01, 0.01, 0.015, 0.02, 0.05, 9999])
# go_velocity = numpy.array([ -0.1, -0.1 , -0.01, -0.01 ,     0,    0, 0.01 , 0.01, 0.1 ,  0.1])
# 
# setting for Real Robot
go_distance = numpy.array([-9999, -0.05, -0.02, -0.01, -0.01, 0.01, 0.01, 0.02, 0.05, 9999])
go_velocity = numpy.array([ -0.5, -0.5 , -0.50, -0.10,     0,    0, 0.10, 0.50, 0.5 ,  0.5])

# go_distance_fast = numpy.array([-9999, -0.02, -0.01, -0.001, -0.0009, 0, 0.001, 0.0011, 0.005, 0.010, 0.020, 9999])
# go_velocity_fast = numpy.array([ -0.1, -0.1 , -0.1 , -0.01 ,       0, 0, 0.01 , 0.01  , 0.015, 0.1  , 0.1  ,  0.1])
go_distance_fast = numpy.array([-9999, -0.2, -0.1, -0.01, -0.009, 0, 0.01, 0.011,  0.05, 0.10, 0.20, 9999])
go_velocity_fast = numpy.array([ -0.2, -0.2, -0.1, -0.01,      0, 0, 0.01,  0.01, 0.015, 0.1 , 0.2 ,  0.2])

turn_angle_quick    = numpy.array([-999, -20.0, -10.0,  -1.0, 0,  1.0, 10.0, 20.0, 999])
turn_velocity_quick = numpy.array([-5.0,  -3.5, -1.00,     0, 0,    0, 1.00,  3.5, 5.0])
move_distance_quick = numpy.array([-999, -0.50, -0.05, -0.02, 0, 0.02, 0.05, 0.50, 999])
move_velocity_quick = numpy.array([-0.5,  -0.5, -0.05,     0, 0,    0, 0.05,  0.5, 0.5]) 
# move_distance = numpy.array([-99999, -1.0, -0.5, -0.10, -0.01, -0.009, 0.009, 0.01, 0.10, 0.5, 1.0, 99999])
# move_velocity = numpy.array([  -0.3, -0.3, -0.1, -0.05, -0.01,      0,     0, 0.01, 0.05, 0.1, 0.3, 0.3  ])
move_distance = numpy.array([-99999, -1.0, -0.5, -0.10, -0.01, -0.01, 0.01, 0.05, 0.10, 0.5, 1.0, 99999])
move_velocity = numpy.array([  -0.3, -0.3, -0.1, -0.05, -0.01,     0,    0, 0.01, 0.05, 0.1, 0.3, 0.3  ])

def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def tangent_angle(u, v):
    i = numpy.inner([u.y, u.x], [v.y, v.x])
    n = linalg.norm([u.y, u.x]) * linalg.norm([v.y, v.x])
    if (n == 0 ):
        return 90
    else:
        c = i / n
    return numpy.rad2deg(numpy.arccos(numpy.clip(c, -1.0, 1.0)))

def MPS_angle(u, v):
    # print(u, v)
    p0 = Point()
    p1 = Point()
    p0.x = 1.0
    p0.y = 0.0
    p1.x = v.x - u.x
    p1.y = v.y - u.y
    return tangent_angle(p1, p0)

class btr_2024(object):
    def __init__(self, topicName):
        self.btrOdometry = Odometry()
        self.topicName = topicName

        # rospy.init_node('btr2024')
        self.sub1 = rospy.Subscriber(self.topicName + "/odom", Odometry, self.robotinoOdometry)
        self.sub3 = rospy.Subscriber(self.topicName + "/btr/centerPoint", Point, self.centerPoint)
        self.sub4 = rospy.Subscriber(self.topicName + "/btr/leftPoint", Point, self.leftPoint)
        self.sub5 = rospy.Subscriber(self.topicName + "/btr/rightPoint", Point, self.rightPoint)
        self.sub6 = rospy.Subscriber(self.topicName + "/btr/forwardPoint", Point, self.forwardPoint)
        self.rate = rospy.Rate(10)
        self.pub1 = rospy.Publisher(self.topicName + "/cmd_vel", Twist, queue_size = 10)

        data = Pose2D()
        self.centerPoint = data
        self.leftPoint = data
        self.rightPoint = data
        self.forwardPoint = data
        self.machineList = ""

        self.startRpLidar()

    def startRpLidar(self):
        # if (self.topicName == ""):
        # setup for RPLidar
        print("scan start:" + self.topicName + '/btr/scan_start')
        rospy.wait_for_service(self.topicName + '/btr/scan_start')
        scan_start = rospy.ServiceProxy(self.topicName + '/btr/scan_start', Empty)
        resp = scan_start()
        print("scan start2")

    def run(self):
        print("run")

    def w_waitOdometry(self):
        seq = self.btrOdometry.header.seq
        while (seq == self.btrOdometry.header.seq):
          self.rate.sleep()

    def w_resetOdometry(self, data):
        odometry = ResetOdometry()
        pose = Pose2D()
        rospy.wait_for_service(self.topicName + '/reset_odometry')
        resetOdometry = rospy.ServiceProxy(self.topicName + '/reset_odometry', ResetOdometry)
        resp = resetOdometry(data.x, data.y, data.theta / 180 * math.pi)
        # print(self.topicName + "/reset_odometry", data.x, data.y, data.theta / 180 * math.pi)

    def w_setVelocity(self, data):
        twist = Twist()
        twist.linear.x = data.x
        twist.linear.y = data.y
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = data.theta
        self.pub1.publish(twist)
        self.rate.sleep()

    def w_robotinoMove(self, x, y, ori = 1000, quick = False):
        global move_distance, move_velocity
        global move_distance_quick, move_velocity_quick
        if (quick == True):
            velocity1 = interpolate.interp1d(move_distance_quick, move_velocity_quick)
        else:
            velocity1 = interpolate.interp1d(move_distance, move_velocity)

        self.w_waitOdometry()
        nowPoint = self.btrOdometry
        ret = True

        theta = nowPoint.pose.pose.position.z / 180 * math.pi 
        if (ori == 1000):
            ori = 0
            turnFlag = False
        else:
            turnFlag = True
        # ori += theta / math.pi * 180
        ori += nowPoint.pose.pose.position.z
        if (ori > 180):
            ori -= 360
        if (ori < -180):
            ori += 360

        # print("robotino:" ,nowPoint)
        # print("theta", ori)
        # print("theta", theta, nowPoint.pose.pose.position.z)
        target_x = x * math.cos(theta) - y * math.sin(theta) + nowPoint.pose.pose.position.x 
        target_y = x * math.sin(theta) + y * math.cos(theta) + nowPoint.pose.pose.position.y
        print(target_x, target_y)
        while True:
            self.w_waitOdometry()
            nowPoint = self.btrOdometry
            theta = (nowPoint.pose.pose.position.z)/ 180 * math.pi
            diff_x1 = (target_x - nowPoint.pose.pose.position.x)
            diff_y1 = (target_y - nowPoint.pose.pose.position.y)
            diff_x = diff_x1 * math.cos(-theta) - diff_y1 * math.sin(-theta)
            diff_y = diff_x1 * math.sin(-theta) + diff_y1 * math.cos(-theta)
            v = Pose2D()
            if (math.isnan(diff_x) or math.isinf(diff_x)):
                v.x = 0
            else:
                v.x = velocity1(diff_x)
            if (math.isnan(diff_y) or math.isinf(diff_y)):
                v.y = 0
            else:
                v.y = velocity1(diff_y)
            # v.x = 0
            if ((abs(diff_x) + abs(diff_y)) < 0.75) and (turnFlag == True):
                v.theta = self.w_turnVelocity(theta / math.pi * 180, ori, quick)
            else:
                v.theta = 0
            theta = nowPoint.pose.pose.position.z / 180 * math.pi

            # print(target_x, nowPoint.pose.pose.position.x)
            # print(target_y, nowPoint.pose.pose.position.y)
            # print(diff_x1, diff_y1)
            # print(diff_x, diff_y)
            if (self.forwardPoint.x < diff_x):
                if (self.forwardPoint.x < 1.0):
                    v.x = v.x / 1.0 * self.forwardPoint.x
                if (self.forwardPoint.x < 0.2):
                    v.x = 0
                    ret = False
                
            self.w_setVelocity(v)
            # print("v: ", v)
            if (v.x == 0) and (v.y == 0):
                # while True:
                    # v.theta = self.w_turnVelocity(theta / math.pi * 180, ori, quick)
                    # theta = self.btrOdometry.pose.pose.position.z / 180 * math.pi
                    # print(v)
                    # self.w_setVelocity(v)
                    # if (v.theta == 0):
                        # break
                if (v.theta == 0):
                    # self.w_setVelocity(v)
                    return ret
                # break
    
    def w_turnVelocity(self, theta, ori, quick = False):
        global turn_angle, turn_velocity
        global turn_angle_quick, turn_velocity_quick
        if (quick == True):
            velocity1 = interpolate.interp1d(turn_angle_quick, turn_velocity_quick)
        else:
            velocity1 = interpolate.interp1d(turn_angle, turn_velocity)
        velocitySign = 0

        diff = ori - theta
        if (diff > 180):
            diff -= 360
        if (diff < -180):
            diff += 360

        if (diff == 0):
            velocitySign = 0
        else:
            velocitySign = diff / abs(diff)

        velocity = abs(velocity1(diff))
        print(theta, ori, diff)
        print(velocity, velocitySign)
        if (quick == True):
            b = 5
        else:
            b = 5
        return velocity * velocitySign / b

    def w_goToInputVelt(self):    # 375mm from left side(= 25 + 50*7)
        # self.w_goToWall(min_mps_distance)
        self.w_goToMPSCenter()
        self.w_robotinoMove(0, 0.030)
        # self.w_goToWall(15)

    def w_goToOutputVelt(self):   # 325mm from left side (= 25 + 50*6)
        # self.w_goToWall(min_mps_distance)
        print("goToMPSCenter")
        self.w_goToMPSCenter()
        print("robotinoMove")
        self.w_robotinoMove(0, -0.030)
        # self.w_goToWall(15)
        print("finished - goToOutputVelt")

    def w_robotinoTurnAbs(self, turnAngle):
        while True:
            print("turnABS")
            nowAngle = self.btrOdometry
            # print(nowAngle.pose.pose.position.z)
            if (nowAngle.header.seq != 0):
                break

        targetAngle = turnAngle - nowAngle.pose.pose.position.z
        if (targetAngle > 180):
            targetAngle -= 360
        if (targetAngle < -180):
            targetAngle += 360

        self.w_robotinoTurn(targetAngle)

    def w_robotinoTurn(self, turnAngle):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)

        while True:
            print("turn")
            nowAngle = self.btrOdometry
            # print(nowAngle.pose.pose.position.z)
            if (nowAngle.header.seq != 0):
                break

        print("position.z:", nowAngle.pose.pose.position.z)
        print("turn Angle:", turnAngle)

        targetAngle = nowAngle.pose.pose.position.z + turnAngle
        if (targetAngle > 180):
            targetAngle -= 360
        if (targetAngle < -180):
            targetAngle += 360

        v = Pose2D()
        v.x = 0
        v.y = 0
        while True:
            diff = (targetAngle - self.btrOdometry.pose.pose.position.z)
            if (diff > 180):
                diff -= 360
            if (diff < -180):
                diff += 360
            v.theta = -velocity1(diff)
            v.theta = self.w_turnVelocity(self.btrOdometry.pose.pose.position.z / 180 * math.pi, targetAngle / 180 * math.pi, quick = False)
            #                     v.theta = self.w_turnVelocity(theta, ori, quick)
            self.w_setVelocity(v)
            # print(targetAngle, self.btrOdometry.pose.pose.position.z, diff, v)
            # if ((-3 < diff) and (diff < 3)):
            if (v.theta == 0):
                break
        v.theta = 0
        self.w_setVelocity(v)
        print("finish")

    def w_goToMPSCenter(self):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)
        rospy.wait_for_service(self.topicName + '/btr_aruco/TagLocation')
        tagInfo = rospy.ServiceProxy(self.topicName + '/btr_aruco/TagLocation', TagLocation)
        tag = tagInfo()
        # print(tag)
        if (tag.ok == False):
            self.w_goToWall(min_mps_distance)
            tag = tagInfo()
            print(tag)
        if (tag.ok == True):
            degree = math.atan(tag.tag_location.y / tag.tag_location.x)
            if (tag.tag_location.y < 0):
                degree = -degree
            self.w_robotinoTurn(degree)
            # print(tag.tag_location.y)
            self.w_robotinoMove(0, tag.tag_location.y)
            for i in range(2):
                # turn parallel for the face of MPS.
                self.w_parallelMPS()
                # goTo at the front of the MPS with 50cm.
                self.w_goToWall(min_mps_distance)
                # go to the front of the MPS.
                self.w_goToMPSCenterLRF()
            #self.w_goToWall(0.17)
            self.w_parallelMPS()

    def w_goToMPSCenterLRF(self):
        global go_distance, go_velocity
                
        velocityY = interpolate.interp1d(move_distance, move_velocity)
        while True:
            dist = (self.rightPoint.y - self.leftPoint.y) / 2 /100
            v = Pose2D()
            v.x = 0
            if (math.isnan(dist) or math.isinf(dist)):
                v.y = 0
            else:
                v.y = velocityY(dist) / 10
            v.theta = 0
            print("MPSCenter:", dist, v.y)
            # if ((-0.001 < v.y) and (v.y < 0.001)):
            #     v.y = 0
            # if (not(math.isnan(dist))):
            self.w_setVelocity(v)
            if (v.y == 0):
                break

    def w_goToWall(self, distance):
        global go_distance_fast, go_velocity_fast
        velocityX = interpolate.interp1d(go_distance_fast, go_velocity_fast)
        print("Wall ", distance)
        while True:
            sensor = self.centerPoint.x
            v = Pose2D()
            if (math.isnan(sensor) or math.isinf(sensor)):
                if (distance < 0.17):
                    v.x = 0
                else:
                    v.x = -0.15
            else:
                v.x = velocityX(sensor - distance)
            v.y = 0
            v.theta = 0
            # print("Wall ", distance, "cm:", sensor, v.x)
            # if ((-0.001 < v.x) and (v.x < 0.001)):
            #     v.x = 0
            # if (not(math.isnan(sensor))):
            self.w_setVelocity(v)
            if (v.x == 0):
                break


    def w_parallelMPS_tag(self):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interpld(turn_angle, turn_velocity)

        self.w_getMPSLocation()
        if (self.MPS_find == True):
            phi = self.resp.tag_location.theta
            angle = math.rag2deg(phi)
            v.theta = velocity1(90 - angle)
            self.w_setVelocity(v)
            
    def w_parallelMPS(self):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)

        while True:
            if (False):
                angle1 = 90
                angle2 = 90
                angle3 = 90
                while ((angle1 == 90) or (angle2 == 90) or (angle3 == 90)):
                    angle1 = MPS_angle(self.leftPoint, self.rightPoint)
                    angle2 = MPS_angle(self.leftPoint, self.centerPoint)
                    angle3 = MPS_angle(self.centerPoint, self.rightPoint)
                    print(angle1)
                angle = (angle1 + angle2 + angle3) / 3.0
                # angle = (angle2 + angle3) / 2
                # angle = angle1
                print(angle, angle1, angle2, angle3)
                #
            else:
                angle = self.centerPoint.z

            v = Pose2D()
            v.x = 0
            v.y = 0
            if (math.isnan(angle)):
                angle = 90
            v.theta = velocity1(90 - angle) / 10 

            print("parallelMPS:", angle, v.theta)
            # v.theta /= 30
            # if ((-0.01 < v.theta) and (v.theta < 0.01)):
            #     v.theta = 0
            # if (not(math.isnan(angle))):
            self.w_setVelocity(v)
            if (v.theta == 0):
                break

    def w_turnClockwise(self):
        print("turnClockWise")
        self.w_goToWall(0.2)
        self.w_robotinoMove(0.0, 0.0, 90, quick = True)
        self.w_robotinoMove(0.7, 0, -90, quick = True)
        self.w_robotinoMove(1.2, 0, -90, quick = True)
        self.w_robotinoMove(0.7, 0, -90, quick = True)


    def w_turnCounterClockwise(self):
        print("turnCounterClockWise")
        self.w_goToWall(0.2)
        self.w_robotinoMove(0.0, 0.0, -90, quick = True)
        self.w_robotinoMove(0.7, 0, 90, quick = True)
        self.w_robotinoMove(1.2, 0, 90, quick = True)
        self.w_robotinoMove(0.7, 0, 90, quick = True)

    def w_getWork(self, y):
        rospy.wait_for_service(self.topicName + '/btr/move_g')
        self.getWork = rospy.ServiceProxy(self.topicName + '/btr/move_g', MuxDelete)
        print("getWork")
        self.resp = self.getWork("{}".format(y))
        print("finish")

    def w_putWork(self, y):
        rospy.wait_for_service(self.topicName + 'btr/move_r')
        self.putWork = rospy.ServiceProxy(self.topicName + '/btr/move_r', MuxDelete)
        print("putWork")
        self.resp = self.putWork("{}".format(y))
        print("finish")

    def w_pick_rs(self):
        rospy.wait_for_service(self.topicName + '/btr/pick_rs')
        self.pick_rs = rospy.ServiceProxy(self.topicName + '/btr/pick_rs', MuxDelete)
        print("pick rs")
        self.resp = self.pick_rs("{}".format(y))
        print("finish")

    def w_put_rs(self):
        rospy.wait_for_service(self.topicName + '/btr/put_rs')
        self.put_rs = rospy.ServiceProxy(self.topicName + '/btr/put_rs', Empty)
        print("put rs")
        self.resp = self.put_rs()
        print("finish")

    def w_looking_for_C0(self):
        rospy.wait_for_service(self.topicName + '/btr/looking_for_C0')
        self.looking_for_C0 = rospy.ServiceProxy(self.topicName + '/btr/looking_for_C0', Empty)
        print("move")
        self.resp = self.looking_for_C0()
        print("finish")

    def w_move_g_C0(self):
        rospy.wait_for_service(self.topicName + '/btr/move_g_C0')
        self.move_g_C0 = rospy.ServiceProxy(self.topicName + '/btr/move_g_C0', Empty)
        print("grasp C0")
        self.resp = self.move_g_C0()
        print("finish")

    def w_move_scan(self):
        rospy.wait_for_service(self.topicName + '/btr/move_s')
        self.move_g_C0 = rospy.ServiceProxy(self.topicName + '/btr/move_s', Empty)
        print("scaning")
        self.resp = self.move_g_C0()
        print("finish")

    def robotinoOdometry(self, data):
        # global self.btrOdometry
        quat = quaternion_to_euler(Quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))
        btrOdometryTmp = data
        btrOdometryTmp.pose.pose.position.x = data.pose.pose.position.x
        btrOdometryTmp.pose.pose.position.y = data.pose.pose.position.y
        btrOdometryTmp.pose.pose.position.z = data.pose.pose.position.z / math.pi * 180
        self.btrOdometry = btrOdometryTmp
        # print(self.btrOdometry.pose.pose.position.x)

    def centerPoint(self, data):
        self.centerPoint = data

    def leftPoint(self, data):
        self.leftPoint = data

    def rightPoint(self, data):
        self.rightPoint = data

    def forwardPoint(self, data):
        self.forwardPoint = data

    def w_getMPSLocation(self):
        rospy.wait_for_service(self.topicName + '/btr_aruco/TagLocation')
        self.getTagLocation = rospy.ServiceProxy(self.topicName + '/btr_aruco/TagLocation', TagLocation)
        print("getTagLocation")
        self.resp = self.getTagLocation()
        self.MPS_find = self.resp.ok
        if (self.resp.ok == False):
            return
        # 350 / 2 = the distance between MPS's tag and MPS's center.
        x = self.resp.tag_location.x * 1.0 + camera_offset + 0.35 / 2.0
        y = self.resp.tag_location.y * 1.0
        phi = self.resp.tag_location.theta
        # print("finish")
        # print(self.resp)

        # while True:
        #     nowPoint = self.btrOdometry
        #     # print(nowAngle.pose.pose.position.z)
        #     if (nowPoint.header.seq != 0):
        #         break
        self.w_waitOdometry()
        nowPoint = self.btrOdometry

        theta = nowPoint.pose.pose.position.z / 180 * math.pi
        target_x = x * math.cos(theta) - y * math.sin(theta) + nowPoint.pose.pose.position.x
        target_y = x * math.sin(theta) + y * math.cos(theta) + nowPoint.pose.pose.position.y
        self.MPS_id = self.resp.tag_id.data
        self.MPS_x = target_x
        self.MPS_y = target_y
        self.MPS_phi = -phi + nowPoint.pose.pose.position.z 
        if ((self.resp.tag_id.data % 2) == 1):
            self.MPS_phi += 180
        # print("Tag:", phi, ", Rob:" , nowPoint.pose.pose.position.z, ", MPS:", self.MPS_phi)
        # print("odom:", nowPoint, ", x:", x, "y:", y) 
        self.MPS_phi = ((self.MPS_phi + 360 * 2) % 360)
        self.MPS_phi = int((self.MPS_phi + 22.5) / 45) * 45
        if self.MPS_x < 0:
            zone = "M"
        else:
            zone = "C"
        zone_x = int(abs(self.MPS_x) / 1.0) + 1
        zone_y = int(abs(self.MPS_y) / 1.0) + 1
        self.MPS_zone = zone + "_Z" + str(zone_x * 10 + zone_y)

    def w_addMPS(self, name, zone, phi):
        self.machineName = name
        self.machineZone = zone
        self.machinexiRotation = phi
        if (len(self.machineList) == 0):
            self.machineList = [[name, zone, phi]]
        else:
            if (not (name in self.machineList)):
                self.machineList.append([name, zone, phi])
# main
#
if __name__ == '__main__':
  args = sys.argv
  challenge = "test"
  if (len(args) == 2):
    challenge = args[1]

  print(challenge)
  challengeFlag = True

  rospy.init_node('btr2024')
  agent = btr_2024("robotino1")
  # while True:
  while not rospy.is_shutdown():

    if (challenge == "test" and challengeFlag):
        agent.run()
        # agent.w_goToOutputVelt()
        agent.w_turnClockwise()
        agent.w_goToInputVelt()
        # w_turnCounterClockwise()
        challengeFlag = False

    agent.rate.sleep()


