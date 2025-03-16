#!/usr/bin/python

import struct
import time
import math
import sys
# import rospy
import rclpy
from rclpy.node import Node
import numpy
from numpy import linalg
from scipy import interpolate
import quaternion
# import tf

# import for kachaka
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand
from rclpy.action import ActionClient


from geometry_msgs.msg import Vector3

from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point, Quaternion, Twist
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, Int16, UInt32, String, \
                         Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, Empty
from nav_msgs.msg import Odometry
import refbox_msgs
# import rcll_btr_msgs
from btr2_msgs.msg import TagInfoResponse, TagLocationResponse
from btr2_msgs.srv import SetOdometry, SetPosition, SetVelocity, \
                              SetDistance, TagInfo,     TagLocation
from rto_msgs.srv import ResetOdometry
from refbox_msgs.msg import MachineReportEntryBTR

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
turn_velocity_quick = numpy.array([-5.0,  -3.5, -0.50,     0, 0,    0, 0.50,  3.5, 5.0])
move_distance_quick = numpy.array([-999, -0.50, -0.05, -0.02, 0, 0.02, 0.05, 0.50, 999])
move_velocity_quick = numpy.array([-0.5,  -0.5, -0.05,     0, 0,    0, 0.05,  0.5, 0.5]) 
# move_distance = numpy.array([-99999, -1.0, -0.5, -0.10, -0.01, -0.009, 0.009, 0.01, 0.10, 0.5, 1.0, 99999])
# move_velocity = numpy.array([  -0.3, -0.3, -0.1, -0.05, -0.01,      0,     0, 0.01, 0.05, 0.1, 0.3, 0.3  ])
move_distance = numpy.array([-99999, -1.0, -0.5, -0.10, -0.01, -0.01, 0.01, 0.05, 0.10, 0.5, 1.0, 99999])
move_velocity = numpy.array([  -0.3, -0.3, -0.1, -0.05, -0.01,     0,    0, 0.01, 0.05, 0.1, 0.3, 0.3  ])

def quaternion_to_euler(quaternion):
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
    roll = numpy.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = numpy.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = numpy.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

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

class btr_2025(Node):
    def __init__(self, topicName):
        self.btrOdometry = Odometry()
        self.topicName = topicName

        # rospy.init_node('btr2025')
        super().__init__('btr2025')
        self.sub1 = self.create_subscription(Odometry, self.topicName + "/odom", self.robotOdometry, 10)
        self.sub3 = self.create_subscription(Point, self.topicName + "/btr/centerPoint", self.centerPoint, 10)
        self.sub4 = self.create_subscription(Point, self.topicName + "/btr/leftPoint", self.leftPoint, 10)
        self.sub5 = self.create_subscription(Point, self.topicName + "/btr/rightPoint", self.rightPoint, 10)
        self.sub6 = self.create_subscription(Point, self.topicName + "/btr/forwardPoint", self.forwardPoint, 10)
        self.pub1 = self.create_publisher(Twist, self.topicName + "/kachaka/manual_control/cmd_vel", 10)
        self.cli1 = self.create_client(Empty, '/btr/scan_start')
        self.cli2 = self.create_client(ResetOdometry, self.topicName + '/reset_odometry')
        self.act1 = ActionClient(self, ExecKachakaCommand, "/kachaka/kachaka_command/execute")
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /btr/scan_start not available, waiting again...')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /reset_odometry not available, waiting again...')
        while not self.act1.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('action /kachaka/kachaka_command/execute  not available, waiting again...')
        self.req1 = Empty.Request()
        self.req2 = ResetOdometry.Request()
        self.act1 = ActionClient(self, ExecKachakaCommand, "/kachaka/kachaka_command/execute")

        data = Pose2D()
        self.centerPoint = data
        self.leftPoint = data
        self.rightPoint = data
        self.forwardPoint = data
        self.machineList = ""

        self.startRpLidar()
        self.Odometry = Odometry()
        self.btrOdometryFlag = False

    def kachaka_move(self, pos_x, pos_y, yaw):
        command = KachakaCommand()
        command.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND
        command.move_to_pose_command_x = pos_x
        command.move_to_pose_command_y = pos_y
        command.move_to_pose_command_yaw = yaw
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = command
        future = self.act1.send_goal_async(goal_msg)
        return future

    def startRpLidar(self):
        # if (self.topicName == ""):
        # setup for RPLidar
        print("scan start:" + self.topicName + '/btr/scan_start')
        # rclpy.wait_for_service(self.topicName + '/btr/scan_start')
        # scan_start = rclpy.ServiceProxy(self.topicName + '/btr/scan_start', Empty)
        self.future = self.cli1.call_async(self.req1)
        rclpy.spin_until_future_complete(self, self.future)

        # resp = scan_start()
        print("scan start2")

    def run(self):
        print("run")

    def w_waitOdometry(self):
        # seq = self.btrOdometry.header.seq
        self.btrOdometryFlag = False
        # while (seq == self.btrOdometry.header.seq):
        # print("waitOdometry")
        while (self.btrOdometryFlag == False):
          # self.rate.sleep()
          # rclpy.create_node('simple_node').create_rate(1)
          rclpy.spin_once(self, timeout_sec = 1.0)
        # print("getOdometry Data")
        # print(self.btrOdometry)

    def w_resetOdometry(self, data):
        # odometry = ResetOdometry()
        pose = Pose2D()
        # rospy.wait_for_service(self.topicName + '/reset_odometry')
        # resetOdometry = rospy.ServiceProxy(self.topicName + '/reset_odometry', ResetOdometry)
        # resp = resetOdometry(data.x, data.y, data.theta / 180 * math.pi)
        self.req2.x = data.x
        self.req2.y = data.y
        self.req2.phi = data.theta / 180 * math.pi
        self.cli2future = self.cli2.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.cli2future, timeout_sec = 1.0)
        # print("resetOdometry: ",data)
        # print(self.topicName + "/reset_odometry", data.x, data.y, data.theta / 180 * math.pi)

    def w_setVelocity(self, data):
        twist = Twist()
        twist.linear.x = data.x * 10
        twist.linear.y = data.y * 10
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = data.theta
        self.pub1.publish(twist)
        print(twist)
        # self.rate.sleep()
        rclpy.spin_once(self, timeout_sec = 1)

    def w_robotMove(self, x, y, ori = 1000, quick = False):
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

        # print("robot:" ,nowPoint)
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
                v.x = 0.0
            else:
                v.x = float(velocity1(diff_x))
            if (math.isnan(diff_y) or math.isinf(diff_y)):
                v.y = 0.0
            else:
                v.y = float(velocity1(diff_y))
            # v.x = 0
            if ((abs(diff_x) + abs(diff_y)) < 0.75) and (turnFlag == True):
                v.theta = self.w_turnVelocity(theta / math.pi * 180, ori, quick)
            else:
                v.theta = 0.0
            theta = nowPoint.pose.pose.position.z / 180 * math.pi

            # print(target_x, nowPoint.pose.pose.position.x)
            # print(target_y, nowPoint.pose.pose.position.y)
            # print(diff_x1, diff_y1)
            # print(diff_x, diff_y)
            if (self.forwardPoint.x < diff_x):
                if (self.forwardPoint.x < 1.0):
                    v.x = v.x / 1.0 * self.forwardPoint.x
                if (self.forwardPoint.x < 0.2):
                    v.x = 0.0
                    ret = False
                
            self.w_setVelocity(v)
            print("v: ", v)
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
        # print(theta, ori, diff)
        # print(velocity, velocitySign)
        if (quick == True):
            b = 5 + 5
        else:
            b = 5
        return velocity * velocitySign / b

    def w_goToInputVelt(self):    # 375mm from left side(= 25 + 50*7)
        # self.w_goToWall(min_mps_distance)
        self.w_goToMPSCenter()
        # self.w_robotMove(0, 0.030)
        self.w_robotMove(0, -0.020)
        # self.w_goToWall(15)

    def w_goToOutputVelt(self):   # 325mm from left side (= 25 + 50*6)
        # self.w_goToWall(min_mps_distance)
        print("goToMPSCenter")
        self.w_goToMPSCenter()
        # print("robotMove")
        # self.w_robotMove(0, -0.030)
        # self.w_goToWall(15)
        print("finished - goToOutputVelt")

    def w_robotTurnAbs(self, turnAngle):
        print("turnABS")
        self.w_waitOdometry()
        nowAngle = self.btrOdometry

        targetAngle = turnAngle - nowAngle.pose.pose.position.z
        if (targetAngle > 180):
            targetAngle -= 360
        if (targetAngle < -180):
            targetAngle += 360

        self.w_robotTurn(targetAngle)

    def w_robotTurn(self, turnAngle):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)

        print("turn")
        self.w_waitOdometry()
        nowAngle = self.btrOdometry

        print("position.z:", nowAngle.pose.pose.position.z)
        print("turn Angle:", turnAngle)

        targetAngle = nowAngle.pose.pose.position.z + turnAngle
        if (targetAngle > 180):
            targetAngle -= 360
        if (targetAngle < -180):
            targetAngle += 360

        v = Pose2D()
        v.x = 0.0
        v.y = 0.0
        print("turn")
        while True:
            self.w_waitOdometry()
            diff = (targetAngle - self.btrOdometry.pose.pose.position.z)
            print(targetAngle, self.btrOdometry.pose.pose.position.z)
            if (diff > 180):
                diff -= 360
            if (diff < -180):
                diff += 360
            v.theta = -velocity1(diff)
            # v.theta = self.w_turnVelocity(self.btrOdometry.pose.pose.position.z / 180 * math.pi, targetAngle / 180 * math.pi, quick = True) # False)
            v.theta = self.w_turnVelocity(self.btrOdometry.pose.pose.position.z, targetAngle, quick = True) #theta, ori, quick)
            print(diff, v.theta)
            self.w_setVelocity(v)
            # print(targetAngle, self.btrOdometry.pose.pose.position.z, diff, v)
            # if ((-3 < diff) and (diff < 3)):
            if (v.theta == 0.0):
                break
        v.theta = 0.0
        self.w_setVelocity(v)
        print("finish")

    def w_goToMPSCenter(self):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)
        rclpy.wait_for_service(self.topicName + '/btr_aruco/TagLocation')
        tagInfo = rclpy.ServiceProxy(self.topicName + '/btr_aruco/TagLocation', TagLocation)
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
            self.w_robotTurn(degree)
            # print(tag.tag_location.y)
            self.w_robotMove(0, tag.tag_location.y)
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
        self.w_robotMove(0.0, 0.0, 90, quick = True)
        self.w_robotMove(0.7, 0, -90, quick = True)
        self.w_robotMove(1.2, 0, -90, quick = True)
        self.w_robotMove(0.7, 0, -90, quick = True)


    def w_turnCounterClockwise(self):
        print("turnCounterClockWise")
        self.w_goToWall(0.2)
        self.w_robotMove(0.0, 0.0, -90, quick = True)
        self.w_robotMove(0.7, 0, 90, quick = True)
        self.w_robotMove(1.2, 0, 90, quick = True)
        self.w_robotMove(0.7, 0, 90, quick = True)

    # def w_getWork(self, y):
    def w_getWork(self):
        rclpy.wait_for_service(self.topicName + '/btr/move_g')
        self.getWork = rclpy.ServiceProxy(self.topicName + '/btr/move_g', Empty)
        print("getWork")
        self.resp = self.getWork()
        print("finish")

    # def w_putWork(self, y):
    def w_putWork(self):
        rclpy.wait_for_service(self.topicName + 'btr/move_r')
        self.putWork = rclpy.ServiceProxy(self.topicName + '/btr/move_r', Empty)
        print("putWork")
        self.resp = self.putWork()
        print("finish")

    def robotOdometry(self, data):
        # global self.btrOdometry
        phi = quaternion_to_euler(data.pose.pose.orientation)[2]
        # print(data.pose.pose.orientation, phi)
        btrOdometryTmp = data
        btrOdometryTmp.pose.pose.position.x = data.pose.pose.position.x
        btrOdometryTmp.pose.pose.position.y = data.pose.pose.position.y
        btrOdometryTmp.pose.pose.position.z = float(phi) # / math.pi * 180
        self.btrOdometry = btrOdometryTmp
        self.btrOdometryFlag = True
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
        rclpy.wait_for_service(self.topicName + '/btr_aruco/TagLocation')
        self.getTagLocation = rclpy.ServiceProxy(self.topicName + '/btr_aruco/TagLocation', TagLocation)
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

    def w_addMPS(self, name, zone, phi = 0):
        self.machineName = name
        self.machineZone = zone
        self.machinexiRotation = phi
        if (len(self.machineList) == 0):
            self.machineList = [[name, zone, phi]]
        else:
            if (not (name in self.machineList)):
                self.machineList.append([name, zone, phi])

def main(args=None):
  rclpy.init(args=args)
  btr2025 = btr_2025(topicName = "")
  # rclpy.spin(btr2025)
  btr2025.run()
  btr2025.w_turnClockwise()
  btr2025.w_goToInputVelt()
  rclpy.shutdown()

# main
#
if __name__ == '__main__':
  args = sys.argv
  challenge = "test"
  if (len(args) == 2):
    challenge = args[1]

  print(challenge)
  challengeFlag = True

  main()
