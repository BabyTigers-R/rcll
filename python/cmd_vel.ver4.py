#!/usr/bin/python

import time
import math
import sys
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
import numpy
from numpy import linalg
from scipy import interpolate
from geometry_msgs.msg import Vector3
from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point, Quaternion, Twist
from std_msgs.msg import Int8, Int16, UInt32, String, \
                         Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, Empty
from nav_msgs.msg import Odometry
import rcll_ros_msgs

# import rcll_btr_msgs
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity, SetDistance
from myagv_msgs.srv import ResetOdometry
from rcll_ros_msgs.msg import MachineReportEntryBTR

machineName = {
    101: "C-CS1-O", 102: "C-CS1-I", 103: "C-CS2-O", 104: "C-CS2-I",
    201: "M-CS1-O", 202: "M-CS1-I", 203: "M-CS2-O", 204: "M-CS2-I",
    111: "C-RS1-O", 112: "C-RS1-I", 113: "C-RS2-O", 114: "C-RS2-I",
    211: "M-RS1-O", 212: "M-RS1-I", 213: "M-RS2-O", 214: "M-RS2-I",
    121: "C-BS-O",  122: "C-BS-I",  221: "M-BS-O",  222: "M-BS-I",
    131: "C-DS-O",  132: "C-DS-I",  231: "M-DS-O",  232: "M-DS-I",
    141: "C-SS-O",  142: "C-SS-I",  241: "M-SS-O",  242: "M-SS-I",
    301: "UMPS-1",  302: "UMPS-2"
}

min_mps_distance = 0.6

# setting for Real Robot
turn_angle    = numpy.array([-999, -25,   -15,   -10,    -5,   -0.1, -0.1, 0.1,   0.1,     5,   10,   15,   25, 999])
turn_velocity = numpy.array([   2, 2.0,    2.0,    1.5, 0.75,   0.02,    0,   0, -0.02, -0.75, -1.5, -2.0, -2.0,  -2])

go_distance = numpy.array([-9999, -0.05, -0.02, -0.01, -0.01, 0.01, 0.01, 0.02, 0.05, 9999])
go_velocity = numpy.array([ -0.5, -0.5 , -0.50, -0.10,    0,    0, 0.10, 0.50, 0.5 ,  0.5])

go_distance_fast = numpy.array([-9999, -0.2, -0.1, -0.01, -0.009, 0, 0.01, 0.011,  0.05, 0.10, 0.20, 9999])
go_velocity_fast = numpy.array([ -0.2, -0.2, -0.1, -0.01,       0, 0, 0.01,  0.01, 0.015, 0.1 , 0.2 ,  0.2])

turn_angle_quick    = numpy.array([-999, -20.0, -10.0,  -1.0, 0,  1.0, 10.0, 20.0, 999])
turn_velocity_quick = numpy.array([-5.0,  -3.5, -0.50,     0, 0,    0, 0.50,  3.5, 5.0])
move_distance_quick = numpy.array([-999, -0.50, -0.05, -0.02, 0, 0.02, 0.05, 0.50, 999])
move_velocity_quick = numpy.array([-0.5,  -0.5, -0.05,     0, 0,    0, 0.05,  0.5, 0.5]) 

move_distance = numpy.array([-99999, -1.0, -0.5, -0.10, -0.01, -0.01, 0.01, 0.05, 0.10, 0.5, 1.0, 99999])
move_velocity = numpy.array([   -0.3, -0.3, -0.1, -0.05, -0.01,     0,    0, 0.01, 0.05, 0.1, 0.3, 0.3  ])

def quaternion_to_euler(quaternion):
    e = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def tangent_angle(u, v):
    i = numpy.inner([u.y, u.x], [v.y, v.x])
    n = linalg.norm([u.y, u.x]) * linalg.norm([v.y, v.x])
    if n == 0:
        return 90.0
    else:
        c = i / n
    return numpy.rad2deg(numpy.arccos(numpy.clip(c, -1.0, 1.0)))

def MPS_angle(u, v):
    p0 = Point()
    p1 = Point()
    p0.x = 1.0
    p0.y = 0.0
    p1.x = v.x - u.x
    p1.y = v.y - u.y
    return tangent_angle(p1, p0)

class rc2026(Node):
    def __init__(self, topicName):
        super().__init__('btr2024')
        self.btrOdometry = Odometry()
        self.topicName = topicName
        self.dt = 0.1

        self.centerPoint = Point()
        self.leftPoint = Point()
        self.rightPoint = Point()
        self.forwardPoint = Point() 

        self.pub1 = self.create_publisher(Twist, "/cmd_vel", 10)

        qos = QoSProfile(depth=10)
        self.sub1 = self.create_subscription(Odometry, self.topicName + "/odom", self.myagvOdometry, qos)
        self.sub3 = self.create_subscription(Point, self.topicName + "/btr/centerPoint", self.centerPointCb, qos)
        self.sub4 = self.create_subscription(Point, self.topicName + "/btr/leftPoint", self.leftPointCb, qos)
        self.sub5 = self.create_subscription(Point, self.topicName + "/btr/rightPoint", self.rightPointCb, qos)
        self.sub6 = self.create_subscription(Point, self.topicName + "/btr/forwardPoint", self.forwardPointCb, qos)
        
        self.startRpLidar()

    def centerPointCb(self, msg):
        self.centerPoint = msg

    def leftPointCb(self, msg):
        self.leftPoint = msg

    def rightPointCb(self, msg):
        self.rightPoint = msg

    def forwardPointCb(self, msg):
        self.forwardPoint = msg

    def myagvOdometry(self, data):
        quat = quaternion_to_euler(Quaternion(x=data.pose.pose.orientation.x, y=data.pose.pose.orientation.y, z=data.pose.pose.orientation.z, w=data.pose.pose.orientation.w))
        btrOdometryTmp = data
        btrOdometryTmp.pose.pose.position.x = data.pose.pose.position.x
        btrOdometryTmp.pose.pose.position.y = data.pose.pose.position.y
        btrOdometryTmp.pose.pose.position.z = quat.z / math.pi * 180
        self.btrOdometry = btrOdometryTmp

    def startRpLidar(self):
        print("scan start:" + self.topicName + '/btr/scan_start')
        client = self.create_client(Empty, self.topicName + '/btr/scan_start')
        while not client.wait_for_service(timeout_sec=1.0):
            print("service not available, waiting again...")
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print("scan start2")

    def run(self):
        print("run")

    def w_waitOdometry(self):
        current_sec = self.btrOdometry.header.stamp.sec
        current_nanosec = self.btrOdometry.header.stamp.nanosec
        while (current_sec == self.btrOdometry.header.stamp.sec and 
               current_nanosec == self.btrOdometry.header.stamp.nanosec):
            rclpy.spin_once(self, timeout_sec=0.01)

    def w_resetOdometry(self, data):
        client = self.create_client(ResetOdometry, self.topicName + '/reset_odometry')
        while not client.wait_for_service(timeout_sec=1.0):
            print("waiting for reset_odometry service...")
        req = ResetOdometry.Request()
        req.x = data.x
        req.y = data.y
        req.theta = data.theta / 180 * math.pi
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def w_setVelocity(self, data):
        twist = Twist()
        twist.linear.x = data.x
        twist.linear.y = data.y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        self.pub1.publish(twist)
        time.sleep(self.dt)

    def w_myagvMove(self, x, y, ori=1000, quick=False):
        global move_distance, move_velocity
        global move_distance_quick, move_velocity_quick
        if quick == True:
            velocity1 = interpolate.interp1d(move_distance_quick, move_velocity_quick)
        else:
            velocity1 = interpolate.interp1d(move_distance, move_velocity)

        self.w_waitOdometry()
        nowPoint = self.btrOdometry
        ret = True

        theta = nowPoint.pose.pose.position.z / 180 * math.pi 
        if ori == 1000:
            ori = 0
            turnFlag = False
        else:
            turnFlag = True
        ori += nowPoint.pose.pose.position.z
        if ori > 180:
            ori -= 360
        if ori < -180:
            ori += 360

        target_x = x * math.cos(theta) - y * math.sin(theta) + nowPoint.pose.pose.position.x 
        target_y = x * math.sin(theta) + y * math.cos(theta) + nowPoint.pose.pose.position.y
        print(target_x, target_y)
        while True:
            self.w_waitOdometry()
            nowPoint = self.btrOdometry
            theta = (nowPoint.pose.pose.position.z) / 180 * math.pi
            diff_x1 = (target_x - nowPoint.pose.pose.position.x)
            diff_y1 = (target_y - nowPoint.pose.pose.position.y)
            diff_x = diff_x1 * math.cos(-theta) - diff_y1 * math.sin(-theta)
            diff_y = diff_x1 * math.sin(-theta) + diff_y1 * math.cos(-theta)
            v = Pose2D()
            if math.isnan(diff_x) or math.isinf(diff_x):
                v.x = 0.0
            else:
                v.x = float(velocity1(diff_x))
            if math.isnan(diff_y) or math.isinf(diff_y):
                v.y = 0.0
            else:
                v.y = float(velocity1(diff_y))
            if ((abs(diff_x) + abs(diff_y)) < 0.75) and (turnFlag == True):
                v.theta = self.w_turnVelocity(theta / math.pi * 180, ori, quick)
            else:
                v.theta = 0.0
            theta = nowPoint.pose.pose.position.z / 180 * math.pi
            
            if self.forwardPoint.x < 1.0 and self.forwardPoint.x > 0.0:
                v.x = v.x * self.forwardPoint.x
                if self.forwardPoint.x < 0.2:
                    v.x = 0.0
                    ret = False
                
            self.w_setVelocity(v)
            if (v.x == 0) and (v.y == 0):
                if v.theta == 0:
                    return ret
            def w_turnVelocity(self, theta, ori, quick=False):
        global turn_angle, turn_velocity
        global turn_angle_quick, turn_velocity_quick
        if quick:
            velocity1 = interpolate.interp1d(
                turn_angle_quick,
                turn_velocity_quick,
                bounds_error=False,
                fill_value=(turn_velocity_quick[0], turn_velocity_quick[-1])
            )
        else:
            velocity1 = interpolate.interp1d(
                turn_angle,
                turn_velocity,
                bounds_error=False,
                fill_value=(turn_velocity[0], turn_velocity[-1])
            )

        diff = ori - theta

        if diff > 180:
            diff -= 360
        if diff < -180:
            diff += 360

        if diff == 0:
            return 0.0

        velocitySign = diff / abs(diff)
        velocity = abs(float(velocity1(diff)))
        b = 5.0
        return velocity * velocitySign / b   
        
    def w_goToInputVelt(self):
        self.w_goToMPSCenter()
        self.w_myagvMove(0.0, -0.020)
        
    def w_goToOutputVelt(self):
        print("goToMPSCenter")
        self.w_goToMPSCenter()
        print("finished - goToOutputVelt")

    def w_myagvTurnAbs(self, turnAngle):
        while True:
            print("turnABS")
            nowAngle = self.btrOdometry
            if nowAngle.header.stamp.sec != 0 or nowAngle.header.stamp.nanosec != 0:
                break
            rclpy.spin_once(self, timeout_sec=0.01)

        targetAngle = turnAngle - nowAngle.pose.pose.position.z
        if targetAngle > 180:
            targetAngle -= 360
        if targetAngle < -180:
            targetAngle += 360

        self.w_myagvTurn(targetAngle)

    def w_myagvTurn(self, turnAngle):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)

        while True:
            print("turn")
            nowAngle = self.btrOdometry
            if nowAngle.header.stamp.sec != 0 or nowAngle.header.stamp.nanosec != 0:
                break
            rclpy.spin_once(self, timeout_sec=0.01)

        print("position.z:", nowAngle.pose.pose.position.z)
        print("turn Angle:", turnAngle)

        targetAngle = nowAngle.pose.pose.position.z + turnAngle
        if targetAngle > 180:
            targetAngle -= 360
        if targetAngle < -180:
            targetAngle += 360

        v = Pose2D()
        v.x = 0.0
        v.y = 0.0
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            diff = (targetAngle - self.btrOdometry.pose.pose.position.z)
            if diff > 180:
                diff -= 360
            if diff < -180:
                diff += 360
            v.theta = self.w_turnVelocity(self.btrOdometry.pose.pose.position.z, targetAngle, quick=True)
            self.w_setVelocity(v)
            if v.theta == 0:
                break
        v.theta = 0.0
        self.w_setVelocity(v)
        print("finish")

    def w_goToMPSCenter(self):
        global turn_angle, turn_velocity
        self.w_myagvTurn(0.0)
        for i in range(2):
            self.w_parallelMPS()
            self.w_goToWall(min_mps_distance)
            self.w_goToMPSCenterLRF()
            self.w_parallelMPS()

    def w_goToMPSCenterLRF(self):
        global move_distance, move_velocity
        velocityY = interpolate.interp1d(move_distance, move_velocity)
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            dist = (self.rightPoint.y - self.leftPoint.y) / 2 / 100
            v = Pose2D()
            v.x = 0.0
            if math.isnan(dist) or math.isinf(dist):
                v.y = 0.0
            else:
                v.y = float(velocityY(dist) / 10)
            v.theta = 0.0
            print("MPSCenter:", dist, v.y)
            self.w_setVelocity(v)
            if v.y == 0:
                break

    def w_goToWall(self, distance):
        global go_distance_fast, go_velocity_fast
        velocityX = interpolate.interp1d(go_distance_fast, go_velocity_fast)
        print("Wall ", distance)
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            sensor = self.centerPoint.x
            v = Pose2D()
            if math.isnan(sensor) or math.isinf(sensor):
                if distance < 0.17:
                    v.x = 0.0
                else:
                    v.x = -0.15
            else:
                v.x = float(velocityX(sensor - distance))
            v.y = 0.0
            v.theta = 0.0
            self.w_setVelocity(v)
            if v.x == 0:
                break        

    def w_parallelMPS(self):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)
        
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            angle1 = MPS_angle(self.leftPoint, self.rightPoint)
            angle2 = MPS_angle(self.leftPoint, self.forwardPoint)
            angle3 = MPS_angle(self.centerPoint, self.rightPoint)
            
            while angle1 == 90.0 or angle2 == 90.0 or angle3 == 90.0:
                rclpy.spin_once(self, timeout_sec=0.01)
                angle1 = MPS_angle(self.leftPoint, self.rightPoint)
                angle2 = MPS_angle(self.leftPoint, self.forwardPoint)
                angle3 = MPS_angle(self.centerPoint, self.rightPoint)
                print("Waiting for valid angles...", angle1)
                
            angle = (angle1 + angle2 + angle3) / 3.0
            print(angle, angle1, angle2, angle3)

            v = Pose2D()
            v.x = 0.0
            v.y = 0.0
            
            if math.isnan(angle):
                angle = 90.0
            v.theta = float(velocity1(90.0 - angle) / 10)

            print("parallelMPS:", angle, v.theta)
            self.w_setVelocity(v)
            if v.theta == 0:
                break

    def w_turnClockwise(self):
        print("turnClockWise")
        self.w_goToWall(0.2)
        self.w_myagvMove(0.0, 0.0, 90, quick=True)
        self.w_myagvMove(0.7, 0.0, -90, quick=True)
        self.w_myagvMove(1.2, 0.0, -90, quick=True)
        self.w_myagvMove(0.7, 0.0, -90, quick=True)

    def w_turnCounterClockwise(self):
        print("turnCounterClockWise")
        self.w_goToWall(0.2)
        self.w_myagvMove(0.0, 0.0, -90, quick=True)
        self.w_myagvMove(0.7, 0.0, 90, quick=True)
        self.w_myagvMove(1.2, 0.0, 90, quick=True)
        self.w_myagvMove(0.7, 0.0, 90, quick=True)

    def w_getWork(self):
        client = self.create_client(Empty, self.topicName + '/btr/scan_start')
        while not client.wait_for_service(timeout_sec=1.0):
            print("waiting ...")
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print("getWork")
        self.getWork_client = self.create_client(Empty, self.topicName + '/btr/move_g')
        future_work = self.getWork_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future_work)
        print("finish")

    def w_putWork(self):
        self.putWork_client = self.create_client(Empty, self.topicName + '/btr/move_r')
        while not self.putWork_client.wait_for_service(timeout_sec=1.0):
            print("waiting for move_r service...")
        future = self.putWork_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        print("finish")


# main
if __name__ == '__main__':
    args = sys.argv
    challenge = "test"
    if len(args) == 2:
        challenge = args[1]

    print(challenge)
    challengeFlag = True

    rclpy.init()
    agent = rc2026("myagv")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(agent, timeout_sec=0.01)

            if challenge == "test" and challengeFlag:
                agent.run()
                agent.w_turnClockwise()
                agent.w_goToInputVelt()
                challengeFlag = False
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()