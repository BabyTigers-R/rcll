#!/usr/bin/python
import struct
import time
import math
import sys
import rospy
import numpy
import cv2
# import robotino2022
import btr_2024
import btr_refbox

# from module_photographer import module_photographer
# from module_work_detect import module_work_detect
# from module_line_detect import module_line_detect
from module_photographer import module_photographer
from module_belt_detect import module_belt_detect
from module_c0_detect import module_c0_detect
#from module_photographer_by_c920 import module_photographer_by_c920
#from module_belt_detect_for_c920 import module_belt_detect_for_c920
# from module_center_of_gravity_detect import module_center_of_gravity_detect

import rcll_ros_msgs
import rcll_btr_msgs
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, PointStamped, Point, Vector3
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, Int16, UInt32, String, \
                         Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse, Empty, EmptyResponse
from nav_msgs.msg import Odometry
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity, \
                              SetDistance
from rcll_ros_msgs.msg import BeaconSignal, ExplorationInfo, \
                              ExplorationSignal, ExplorationZone, GameState, \
                              LightSpec, MachineInfo, Machine, \
                              MachineReportEntry, MachineReportEntryBTR, \
                              MachineReportInfo, OrderInfo, Order, \
                              ProductColor, RingInfo, Ring, Team, Time, \
                              NavigationRoutes, Route
from rcll_ros_msgs.srv import SendBeaconSignal, SendMachineReport, \
                              SendMachineReportBTR, SendPrepareMachine

from module_detector import module_detector

TEAMNAME = "BabyTigers-R"

### for Challenge Track
# FIELDMINX = -5
# FIELDMAXX = -1
# FIELDMINY =  1
# FIELDMAXY =  5
FIELDMINX = -5
# FIELDMAXX = -1
FIELDMAXX =  5
FIELDMINY =  1
FIELDMAXY =  5

### for Main Track
# FIELDMINX = -7
# FIELDMAXX = 7
# FIELDMINY = 1
# FIELDMAXY = 8
FIELDSIZEX = (FIELDMAXX - FIELDMINX) + 1
FIELDSIZEY = (FIELDMAXY - FIELDMINY) + 1
FIELDSIZE = FIELDSIZEX * FIELDSIZEY
MAXSTEP = 999

FalseValue = 9999

zoneX = { "S11" : -0.5,  "S21" : -1.5,  "S31" : -2.5,  "S41" : -3.5,  "S51" : -4.5,
          "S12" : -0.5,  "S22" : -1.5,  "S32" : -2.5,  "S42" : -3.5,  "S52" : -4.5,
          "S13" : -0.5,  "S23" : -1.5,  "S33" : -2.5,  "S43" : -3.5,  "S53" : -4.5,
          "S14" : -0.5,  "S24" : -1.5,  "S34" : -2.5,  "S44" : -3.5,  "S54" : -4.5,
          "S15" : -0.5,  "S25" : -1.5,  "S35" : -2.5,  "S45" : -3.5,  "S55" : -4.5,
           "11" :  0.5,   "21" :  1.5,   "31" :  2.5,   "41" :  3.5,   "51" :  4.5,   "61" :  5.5,   "71" :  6.5,
           "12" :  0.5,   "22" :  1.5,   "32" :  2.5,   "42" :  3.5,   "52" :  4.5,   "62" :  5.5,   "72" :  6.5,
           "13" :  0.5,   "23" :  1.5,   "33" :  2.5,   "43" :  3.5,   "53" :  4.5,   "63" :  5.5,   "73" :  6.5,
           "14" :  0.5,   "24" :  1.5,   "34" :  2.5,   "44" :  3.5,   "54" :  4.5,   "64" :  5.5,   "74" :  6.5,
           "15" :  0.5,   "25" :  1.5,   "35" :  2.5,   "45" :  3.5,   "55" :  4.5,   "65" :  5.5,   "75" :  6.5,
           "16" :  0.5,   "26" :  1.5,   "36" :  2.5,   "46" :  3.5,   "56" :  4.5,   "66" :  5.5,   "76" :  6.5,
           "17" :  0.5,   "27" :  1.5,   "37" :  2.5,   "47" :  3.5,   "57" :  4.5,   "67" :  5.5,   "77" :  6.5,
           "18" :  0.5,   "28" :  1.5,   "38" :  2.5,   "48" :  3.5,   "58" :  4.5,   "68" :  5.5,   "78" :  6.5,
         "1011" : -0.5, "1021" : -1.5, "1031" : -2.5, "1041" : -3.5, "1051" : -4.5, "1061" : -5.5, "1071" : -6.5,
         "1012" : -0.5, "1022" : -1.5, "1032" : -2.5, "1042" : -3.5, "1052" : -4.5, "1062" : -5.5, "1072" : -6.5,
         "1013" : -0.5, "1023" : -1.5, "1033" : -2.5, "1043" : -3.5, "1053" : -4.5, "1063" : -5.5, "1073" : -6.5,
         "1014" : -0.5, "1024" : -1.5, "1034" : -2.5, "1044" : -3.5, "1054" : -4.5, "1064" : -5.5, "1074" : -6.5,
         "1015" : -0.5, "1025" : -1.5, "1035" : -2.5, "1045" : -3.5, "1055" : -4.5, "1065" : -5.5, "1075" : -6.5,
         "1016" : -0.5, "1026" : -1.5, "1036" : -2.5, "1046" : -3.5, "1056" : -4.5, "1066" : -5.5, "1076" : -6.5,
         "1017" : -0.5, "1027" : -1.5, "1037" : -2.5, "1047" : -3.5, "1057" : -4.5, "1067" : -5.5, "1077" : -6.5,
         "1018" : -0.5, "1028" : -1.5, "1038" : -2.5, "1048" : -3.5, "1058" : -4.5, "1068" : -5.5, "1078" : -6.5}

zoneY = { "S11" :  0.5,  "S21" :  0.5,  "S31" :  0.5,  "S41" :  0.5,  "S51" :  0.5,
          "S12" :  1.5,  "S22" :  1.5,  "S32" :  1.5,  "S42" :  1.5,  "S52" :  1.5,
          "S13" :  2.5,  "S23" :  2.5,  "S33" :  2.5,  "S43" :  2.5,  "S53" :  2.5,
          "S14" :  3.5,  "S24" :  3.5,  "S34" :  3.5,  "S44" :  3.5,  "S54" :  3.5,
          "S15" :  4.5,  "S25" :  4.5,  "S35" :  4.5,  "S45" :  4.5,  "S55" :  4.5,
           "11" :  0.5,   "21" :  0.5,   "31" :  0.5,   "41" :  0.5,   "51" :  0.5,   "61" : 0.5,   "71" : 0.5,
           "12" :  1.5,   "22" :  1.5,   "32" :  1.5,   "42" :  1.5,   "52" :  1.5,   "62" : 1.5,   "72" : 1.5,
           "13" :  2.5,   "23" :  2.5,   "33" :  2.5,   "43" :  2.5,   "53" :  2.5,   "63" : 2.5,   "73" : 2.5,
           "14" :  3.5,   "24" :  3.5,   "34" :  3.5,   "44" :  3.5,   "54" :  3.5,   "64" : 3.5,   "74" : 3.5,
           "15" :  4.5,   "25" :  4.5,   "35" :  4.5,   "45" :  4.5,   "55" :  4.5,   "65" : 4.5,   "75" : 4.5,
           "16" :  5.5,   "26" :  5.5,   "36" :  5.5,   "46" :  5.5,   "56" :  5.5,   "66" : 5.5,   "76" : 5.5,
           "17" :  6.5,   "27" :  6.5,   "37" :  6.5,   "47" :  6.5,   "57" :  6.5,   "67" : 6.5,   "77" : 6.5,
           "18" :  7.5,   "28" :  7.5,   "38" :  7.5,   "48" :  7.5,   "58" :  7.5,   "68" : 7.5,   "78" : 7.5,
         "1011" :  0.5, "1021" :  0.5, "1031" :  0.5, "1041" :  0.5, "1051" :  0.5, "1061" : 0.5, "1071" : 0.5,
         "1012" :  1.5, "1022" :  1.5, "1032" :  1.5, "1042" :  1.5, "1052" :  1.5, "1062" : 1.5, "1072" : 1.5,
         "1013" :  2.5, "1023" :  2.5, "1033" :  2.5, "1043" :  2.5, "1053" :  2.5, "1063" : 2.5, "1073" : 2.5,
         "1014" :  3.5, "1024" :  3.5, "1034" :  3.5, "1044" :  3.5, "1054" :  3.5, "1064" : 3.5, "1074" : 3.5,
         "1015" :  4.5, "1025" :  4.5, "1035" :  4.5, "1045" :  4.5, "1055" :  4.5, "1065" : 4.5, "1075" : 4.5,
         "1016" :  5.5, "1026" :  5.5, "1036" :  5.5, "1046" :  5.5, "1056" :  5.5, "1066" : 5.5, "1076" : 5.5,
         "1017" :  6.5, "1027" :  6.5, "1037" :  6.5, "1047" :  6.5, "1057" :  6.5, "1067" : 6.5, "1077" : 6.5,
         "1018" :  7.5, "1028" :  7.5, "1038" :  7.5, "1048" :  7.5, "1058" :  7.5, "1068" : 7.5, "1078" : 7.5}
inputX = { 0: 1.0, 45: 0.5, 90:   0, 135: -0.5, 180: -1.0, 225: -0.5, 270:    0, 315:  0.5, 360: 1.0}
inputY = { 0:   0, 45: 0.5, 90: 1.0, 135:  0.5, 180:    0, 225: -0.5, 270: -1.0, 315: -0.5, 360:   0}
outputX = {  0: inputX[180],  45: inputX[225],  90: inputX[270], 135: inputX[315],
           180: inputX[  0], 225: inputX[ 45], 270: inputX[ 90], 315: inputX[135]}
outputY = {  0: inputY[180],  45: inputY[225],  90: inputY[270], 135: inputY[315],
           180: inputY[  0], 225: inputY[ 45], 270: inputY[ 90], 315: inputY[135]}
machineName = { 101 : "C-CS1-O", 102 : "C-CS1-I", 103 : "C-CS2-O", 104 : "C-CS2-I",
                201 : "M-CS1-O", 202 : "M-CS1-I", 203 : "M-CS2-O", 204 : "M-CS2-I",
                111 : "C-RS1-O", 112 : "C-RS1-I", 113 : "C-RS2-O", 114 : "C-RS2-I",
                211 : "M-RS1-O", 212 : "M-RS1-I", 213 : "M-RS2-O", 214 : "M-RS2-I",
                121 : "C-BS-O",  122 : "C-BS-I",  221 : "M-BS-O",  222 : "M-BS-I",
                131 : "C-DS-O",  132 : "C-DS-I",  231 : "M-DS-O",  232 : "M-DS-I",
                141 : "C-SS-O",  142 : "C-SS-I",  241 : "M-SS-O",  242 : "M-SS-I",
                301 : "UMPS-1",  302 : "UMPS-2" }
oldTheta = 0


CS_OP_RETRIEVE_CAP = 1
CS_OP_MOUNT_CAP = 2
BS_SIDE_INPUT = 1
BS_SIDE_OUTPUT = 2


class btr_rcll(object):
    def __init__(self, teamName = "BabyTigers-R", robotNum = 0, gazeboFlag = False, refbox = None):
        self.topicName = ""
        self.gazeboFlag = gazeboFlag
        self.robotNum = robotNum
        if (gazeboFlag):
            self.topicName = "/robotino" + str(robotNum)
        else:
            self.pg = module_photographer()
            self.bd = module_belt_detect()
            self.c0d = module_c0_detect()

        if (refbox == None):
            print("please set refbox arg")
            # break
            exit()
        self.refbox = refbox

        self.btrOdometry = Odometry()
        self.btrBeaconCounter = 0
        self.btrVelocity = Float32MultiArray()

        self.btrField = [[0 for y in range(FIELDMINY, FIELDMAXY + 1)] for x in range(FIELDMINX, FIELDMAXX + 1)]

        self. nodeName = "btr_2024_" + str(robotNum)
        rospy.init_node(self.nodeName)
        self.sub01 = rospy.Subscriber(self.topicName + "/odom", Odometry, self.robotinoOdometry)
        rate = rospy.Rate(10)

        self.machineReport = MachineReportEntryBTR()
        self.prepareMachine = SendPrepareMachine()

        print("gazeboFlag: " + str(gazeboFlag))
        print("topicName: " + self.topicName)
        self.btrRobotino = btr_2024.btr_2024(self.topicName)

    def challenge(self, challenge = "test"):
        print("rcll_btr2024: ", challenge)
        if (challenge == "reset"):
            self.goToPoint(-3.5,  1.5, 90)
            self.goToPoint(-3.5,  0.5, 90)
            # self.goToPoint(pose.x, pose.y, pose.theta)
            exit()

        pose = Pose2D()
        #
        ## setting for Challenge Track
        pose.x = -1.0 * self.robotNum - 1.5
        pose.y = 0.5
        pose.theta = 90
        if (challenge == "grasping"):
            startX =     [ -0.5, -4.5, -0.5]
            startY =     [  0.5,  1.5,  4.5]
            startTheta = [   90,   90,  180]
            pose.x = startX[self.robotNum - 1]
            pose.y = startY[self.robotNum - 1]
            pose.theta = startTheta[self.robotNum - 1]
        if (challenge == "navigation"):
            pose.x = zoneX["S31"]
            pose.y = zoneY["S31"]
            pose.theta = 90
        if (challenge == "rcll" or challenge == "test"):
            pose.x = -(pose.x - 2.0)
        #
        # setting for Main Track
        print("Challenge: " + challenge)
        if (challenge == "main_exploration" or challenge == "main_production"):
            pose.x = 3.5 + self.robotNum
        
        # 
        # this year(2024), both side are used.
        print("Team Color: ", self.refbox.teamColor)
        if (self.refbox.teamColor == 1):
            pose.x = -pose.x

        print(pose.x, pose.y, pose.theta)
        self.btrRobotino.w_resetOdometry(pose)
        # time.sleep(3)
        self.btrRobotino.w_waitOdometry()

        print("btr_rcll2024: ", challenge)
        # self.challengeFlag = True
        self.initField()

        if (challenge == "exploration"):
            self.challenge_exploration()
        if (challenge == "main_exploration"):
            self.main_exploration()
        if (challenge == "gripping"):
            self.challenge_gripping()
        if (challenge == "graspingTest"):
            self.challenge_graspingTest()
        if (challenge == "grasping"):
            self.challenge_grasping()
        if (challenge == "navigationTest"):
            self.challenge_navigationTest()
        if (challenge == "navigation"):
            self.challenge_navigation()
        if (challenge == "beacon"):
            self.challenge_beacon()
        if (challenge == "production"):
            self.startProduction();

    def challenge_exploration(self):
        # goTo S32
        self.goToPoint(zoneX["S32"], zoneY["S32"], 90)
        for i in range(5):
            self.w_findMPS()
            self.btrRobotino.w_robotinoTurnAbs(45 * i)
        # goTo S34
        navPoint = Pose2D()
        for ZONE in ["S34", "S44", "S42", "S22", "S24", "S32"]:
            navPoint.x = zoneX[ZONE]
            navPoint.y = zoneY[ZONE]
            navPoint.theta = 90
            self.navToPoint(navPoint)
            for i in range(9):
                self.w_findMPS()
                self.btrRobotino.w_robotinoTurnAbs(45 * i)

    def main_exploration(self):
        # goTo some points
        navPoint = Pose2D()
        for ZONE in ["52", "22", "26", "66", "62", "44"]:
            navPoint.x = int(ZONE) / 10 # zoneX[ZONE]
            navPoint.y = int(ZONE) - (int(int(ZONE) / 10)) * 10 # zoneY[ZONE]
            navPoint.theta = 90
            print("go To ", ZONE)
            self.navToPoint(navPoint)
            for i in range(9):
                self.w_findMPS()
                self.btrRobotino.w_robotinoTurnAbs(45 * i)

    def challenge_gripping(self):
        slotNo = 3
        if (True):
            moveGo   = [-0.100, -0.220, -0.305]
            moveBack = [ 0.105,  0.220,  0.310]
            self.btrRobotino.w_goToInputVelt()
            self.btrRobotino.w_robotinoMove(0,  moveGo[  slotNo - 1])
            self.btrRobotino.w_getWork()
            # self.btrRobotino.w_goToInputVelt()
            self.btrRobotino.w_robotinoMove(0,  moveBack[slotNo - 1])
            self.btrRobotino.w_putWork()

    def challenge_graspingTest(self):
        self.getWorkOnShelf()
        self.putWorkOnConveyor()

        # self.startGrasping()
        # pg = module_photographer()
        # bd = module_belt_detect()
        # c0d = module_c0_detect()
        # self.bringC0(self.pg, self.c0d)
        if (True):
            return 0

        self.bringC0()
        print("goToWall")
        self.btrRobotino.w_goToWall(0.35)
        self.btrRobotino.w_robotinoMove(0, 0.05)
        print("parallelMPS")
        self.btrRobotino.w_parallelMPS()
        print("goToWall")
        
        self.btrRobotino.w_goToWall(0.22)
        belt_position_error = self.adjustment(self.pg, self.bd, True)
        self.btrRobotino.w_putWork()

    def getWorkOnShelf(self):
        # self.startGrasping()
        self.bringC0()
           
    def putWorkOnConveyor(self):
        print("goToInputVelt")
        self.btrRobotino.w_goToInputVelt()
        print("goToWall")
        self.btrRobotino.w_goToWall(0.35)
        self.btrRobotino.w_robotinoMove(0, 0.05)
        print("parallelMPS")
        self.btrRobotino.w_parallelMPS()
        print("goToWall")
        self.btrRobotino.w_goToWall(0.22)
        belt_position_error = self.adjustment(self.pg, self.bd, True)

        self.btrRobotino.w_putWork()

    def getWorkOnConveyor(self):
        print("goToOutputVelt")
        self.btrRobotino.w_goToOutputVelt()
        print("goToWall")
        self.btrRobotino.w_goToWall(0.35)
        self.btrRobotino.w_robotinoMove(0, -0.05)
        print("parallelMPS")
        self.btrRobotino.w_parallelMPS()
        print("goToWall")
        self.btrRobotino.w_goToWall(0.26)
        belt_position_error = self.adjustment(self.pg, self.bd, True)
        self.btrRobotino.w_getWork()

    def putWorkOnSlide(self):
        self.btrRobotino.w_goToInputVelt()
        print("goToWall")
        self.btrRobotino.w_goToWall(0.35)
        self.btrRobotino.w_robotinoMove(0, 0.05)
        print("parallelMPS")
        self.btrRobotino.w_parallelMPS()
        print("goToWall")
        self.btrRobotino.w_goToWall(0.22)
        self.btrRobotino.w_robotinoMove(0, -0.28)
        self.btrRobotino.w_putWork()

    def bringC0(self):
        print("goToInputVelt")
        self.btrRobotino.w_goToInputVelt()
        print("goToWall")
        self.btrRobotino.w_goToWall(0.35)
        self.btrRobotino.w_robotinoMove(0, 0.05)
        print("parallelMPS")
        self.btrRobotino.w_parallelMPS()
        print("goToWall")
        self.btrRobotino.w_goToWall(0.27)

        self.btrRobotino.w_robotinoMove(0, -0.22)
        belt_position_error = self.adjustment(self.pg, self.c0d, False)
        self.btrRobotino.w_getWork()

        # back to Input
        self.btrRobotino.w_robotinoMove(0, 0.22)
        print("parallelMPS")
        self.btrRobotino.w_parallelMPS()
        print("goToInputVelt")
        self.btrRobotino.w_goToInputVelt()

    def MPS2Zone(self, MPSName):
        while (self.refbox.refboxMachineInfoFlag == False):
            self.btrRobotino.rate.sleep()

        for i in self.refbox.refboxMachineInfo.machines:
            # print("MPS2Zone: ", MPSName, i.name, MPSName == i.name, i.zone)
            if (i.name == MPSName):
                return i.zone
        return False

    def MPS2Angle(self, MPSName):
        while (self.refbox.refboxMachineInfoFlag == False):
            self.btrRobotino.rate.sleep()

        for i in self.refbox.refboxMachineInfo.machines:
            print("MPS2Angle: ", MPSName, i.name, MPSName == i.name, i.rotation)
            if (i.name == MPSName):
                try:
                    if (i.rotation):
                        return i.rotation
                    return i.rotation
                except AttributeError:
                    print("there is no rotation information")
                    return False
        return False

    def MPS2Point(self, MPSName, MPSSide):
        MPSZone = self.MPS2Zone(MPSName)
        MPSAngle = FalseValue
        while (MPSAngle == FalseValue):
            MPSAngle = self.MPS2Angle(MPSName)
            print("MPS2Point: ", MPSName, MPSAngle, MPSAngle == FalseValue)
        print("MPS2Point: ", MPSZone, MPSAngle, MPSSide)

        return self.Zone2XYT(MPSZone, MPSAngle, MPSSide)

    def Zone2XYT(self, MPSZone, MPSAngle, MPSSide = "input"):
        MPSPose = Pose2D()
        if (MPSZone == FalseValue):
            return False
        if (MPSAngle == FalseValue):
            return False
        MPSZonePoint = self.zoneToXY(MPSZone)
        if (MPSSide == "input"):
            MPSPose.x = MPSZonePoint.x + inputX[MPSAngle]
            MPSPose.y = MPSZonePoint.y + inputY[MPSAngle]
            MPSPose.theta = MPSAngle + 180
        elif (MPSSide == "output"):
            MPSPose.x = MPSZonePoint.x + outputX[MPSAngle]
            MPSPose.y = MPSZonePoint.y + outputY[MPSAngle]
            MPSPose.theta = MPSAngle
        else:
            MPSPose.x = MPSZonePoint.x
            MPSPose.y = MPSZonePoint.y
            MPSPose.theta = MPSAngle

        while(MPSPose.theta >= 360):
            MPSPose.theta -= 360
        return MPSPose

    def challenge_grasping(self):
        self.startGrasping()

    def challenge_navigationTest(self):
        self.startNavigation()

    def challenge_navigation(self):
        while (True):
            if (self.refbox.refboxMachineInfoFlag and self.refbox.refboxNavigationRoutesFlag):
                self.startNavigation()
                break
            else:
                print("wait for navigation")
                self.btrRobotino.rate.sleep()

    def challenge_beacon(self):
        self.refbox.sendBeacon()
        print("Game status is ", self.refbox.refboxGamePhase)

    def challenge_clockwise(self):
        # self.btrRobotino.w_turnClockwise()
        # not go to wall
        self.btrRobotino.w_robotinoMove(0.0, 0.0, 90, quick = True)
        self.btrRobotino.w_robotinoMove(0.7, 0, -90, quick = True)
        self.btrRobotino.w_robotinoMove(1.2, 0, -90, quick = True)
        self.btrRobotino.w_robotinoMove(0.7, 0, -90, quick = True)

    def challenge_camera(self):
        self.btrRobotino.w_goToInputVelt()
        self.btrRobotino.w_parallelMPS()
        self.btrRobotino.w_goToWall(0.4)
#
#
#
    def robotinoOdometry(self, data):
        # global btrOdometry, btrBeaconCounter
        self.btrOdometry = data
        self.refbox.robotinoOdometry(data)
        # print(data)
        ## self.btrOdometry.pose.pose.position.z = quat.z / math.pi * 180
        # self.btrOdometry.pose.pose.position.z = self.btrOdometry.pose.pose.position.z # / math.pi * 180

#
#
#

    def w_findMPS(self):
        self.btrRobotino.w_getMPSLocation()
        if (self.btrRobotino.MPS_find == True and self.btrRobotino.MPS_id > 0):
            # print(self.btrRobotino.MPS_id)
            if not (self.btrRobotino.MPS_id in machineName):
                print(self.btrRobotino.MPS_id, "is not ID?")
                return False
            name = machineName[self.btrRobotino.MPS_id]
            zone = self.btrRobotino.MPS_zone
            self.machineReport.name = name[0: len(name) - 2]
            if (name[4 : 5] == "-"):
                self.machineReport.type = name[2 : 4]
            else:
                self.machineReport.type = name[2 : 5]
            zone = int(self.btrRobotino.MPS_zone[3 : 5])
            if (self.btrRobotino.MPS_zone[0: 1] == "M"):
                zone = -zone # + 1000
            # print(self.btrRobotino.MPS_zone, zone)

            self.machineReport.zone = zone
            self.machineReport.rotation = self.btrRobotino.MPS_phi
            self.refbox.sendMachineReport(self.machineReport)

            self.btrRobotino.w_addMPS(name, zone, self.btrRobotino.MPS_phi)
        return self.btrRobotino.MPS_find

    def goToPoint(self, x, y, phi):
        self.btrRobotino.w_robotinoMove(0, 0)
        self.btrRobotino.w_waitOdometry()
        nowX = self.btrOdometry.pose.pose.position.x
        nowY = self.btrOdometry.pose.pose.position.y
        nowPhi = self.btrOdometry.pose.pose.position.z
        dist = ((x - nowX)**2 + (y - nowY)**2) **0.5
        print(nowX, nowY, "=>", x, y)
        print(dist)
        if (dist > 0.30):
            turn = numpy.rad2deg(numpy.arctan2(y - nowY, x - nowX))
            print(nowX, nowY, nowPhi, "=>", x, y, phi)
            # print(turn, turn - nowPhi)
            # self.btrRobotino.w_robotinoTurn(turn - nowPhi)
            self.btrRobotino.w_robotinoMove(0, 0, turn - nowPhi, quick = False)
            # self.btrRobotino.w_robotinoMove(dist, 0)
            nowPhi = self.btrOdometry.pose.pose.position.z
            self.btrRobotino.w_robotinoMove(dist, 0, phi - nowPhi, quick = True)
        else:
            print("dist <= 0.30")
            moveX = x - nowX
            moveY = y - nowY
            # print(moveX, moveY, nowPhi)
            rad = math.radians(nowPhi)
            distX = moveX * math.cos(-rad) - moveY * math.sin(-rad)
            distY = moveX * math.sin(-rad) + moveY * math.cos(-rad)
            # self.btrRobotino.w_robotinoMove(distX, distY)
            self.btrRobotino.w_robotinoMove(distX, distY, phi - nowPhi, quick = False)
        nowPhi = self.btrOdometry.pose.pose.position.z
        # print(phi, phi - nowPhi)
        # self.btrRobotino.w_robotinoTurn(phi - nowPhi)
        # self.btrRobotino.w_robotinoMove(x, y)
        # self.btrRobotino.w_robotinoTurn(phi)

    # 
    # challenge program
    #

    def startGrasping(self):
        # pg = module_photographer()
        # bd = module_belt_detect()

        for _ in range(3):
            print("{} / 3 repeation".format(_+1))
            # self.challengeFlag = False

            print("goToOutputVelt")
            self.btrRobotino.w_goToOutputVelt()
            print("goToWall")
            self.btrRobotino.w_goToWall(0.35)
            self.btrRobotino.w_robotinoMove(0, -0.05)
            print("parallelMPS")
            self.btrRobotino.w_parallelMPS()
            print("goToWall")
            self.btrRobotino.w_goToWall(0.26)

            belt_position_error = self.adjustment(self.pg, self.bd, True)
            self.btrRobotino.w_getWork()
            # break

            if (self.robotNum != 2):
                self.btrRobotino.w_turnClockwise()
            else:
                self.btrRobotino.w_turnCounterClockwise()

            print("goToInputVelt")
            self.btrRobotino.w_goToInputVelt()
            print("goToWall")
            self.btrRobotino.w_goToWall(0.35)
            # self.btrRobotino.w_robotinoMove(0, 0.05)
            print("parallelMPS")
            self.btrRobotino.w_parallelMPS()
            print("goToWall")
            self.btrRobotino.w_goToWall(0.26)

            belt_position_error = self.adjustment(self.pg, self.bd, True)
            self.btrRobotino.w_putWork()

            if (self.robotNum != 2):
                self.btrRobotino.w_turnCounterClockwise()
            else:
                self.btrRobotino.w_turnClockwise()


    def adjustment(self, pg, detector, belt):
        for _ in range(25):
            gray, bg_removed = pg() # In case of "True" realsense detects conver belt.
            if belt:
                position_error = detector(gray)
            else:
                position_error = detector(bg_removed)
            time.sleep(0.5)

            print(position_error)

            if position_error == 2:
                pass
            elif position_error == -1:
                self.btrRobotino.w_robotinoMove(0, -0.015)
            elif position_error == 1:
                self.btrRobotino.w_robotinoMove(0, 0.015)
            elif position_error == 0:
                break

            print("parallelMPS")
            self.btrRobotino.w_parallelMPS()

        return position_error

    def initField(self):
        # global btrField
        # btrField = [[0 for y in range(FIELDSIZEY)] for x in range(FIELDSIZEX)]
        self.btrField = [[0 for x in range(FIELDSIZEX)] for y in range(FIELDSIZEY)]
        for zone in range(2):
            for x in range(1, 8):
                for y in range(1, 8):
                    zoneName = str(zone * 100 + x) + str(y)
                    zoneX[zoneName] = (x - 0.5) * (-zone * 2 + 1)
                    zoneY[zoneName] = y - 0.5
                    # print(zoneName, zoneX[zoneName], zoneY[zoneName])
        #
        # this field is [y][x]
        # but game field is from -5 to -1
        # so when you use this variable, please shift argment + 5.
        #   (-5, 5) => (0, 4)
        #   (-5 ,1) => (0, 0)

    def setField(self, x, y, number):
        global FIELDMINX, FIELDMINY
        # print(x, y, FIELDMINX, FIELDMINY)
        if (x < FIELDMINX or x > FIELDMAXX or y < FIELDMINY or y > FIELDMAXY):
            print("setField - out of field: ", x, y, number)
            return
        self.btrField[y - FIELDMINY][x - FIELDMINX] = number

    def getField(self, x, y):
        global FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
        if (int(x) < FIELDMINX or FIELDMAXX < int(x) or int(y) < FIELDMINY or FIELDMAXY < int(y)):
            # print("getField range over: ", x, y)
            return MAXSTEP 
        return self.btrField[int(y) - FIELDMINY][int(x) - FIELDMINX]

    def zoneToXY(self, zone):
        point = Pose2D()
        point.y = abs(int(zone)) % 10
        point.x = (abs(int(zone)) % 100) // 10
        # if (zone < 0):
        #     point.x = -point.x
        if zone > 1000:
          point.x = -point.x
        print(zone, point.x, point.y)
        return point

    def setMPStoField(self):
        # global btrField
        point = Pose2D()
     
        if (len(self.refbox.refboxMachineInfo.machines) > 0):
            self.btrRobotino.machineList = ""
            for machine in self.refbox.refboxMachineInfo.machines:
                self.btrRobotino.w_addMPS(machine.name, machine.zone)

        # print(self.btrRobotino.machineList)
        for machine in self.btrRobotino.machineList:
            # print(machine)
            point = self.zoneToXY(machine[1])
            print("setMPS: ", machine[0], machine[1], point.x, point.y)
            if (point.x == 0 and point.y == 0):
                print("received NULL data for MPS", machine[0])
            else:
                self.setField(point.x, point.y, MAXSTEP)

    def getStep(self, x, y):
        global FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
        if ((x < FIELDMINX or FIELDMAXX < x) or (y < FIELDMINY or FIELDMAXY < y)):
            return MAXSTEP

        step = self.getField(x, y)
        if (step == 0):
            step = MAXSTEP
        return step

    def wallCheck(self, x, y, dx, dy):
        notWallFlag = True
        if (FIELDMINX == -5):
            # magenta side
            print(x, y, dx, dy)
            if ((x == -5 and y == 1) and (dx ==  0 and dy ==  1)):
                notWallFlag = False
            if ((x == -4 and y == 1) and (dx ==  0 and dy ==  1)):
                notWallFlag = False
            if ((x == -3 and y == 1) and (dx ==  1 and dy ==  0)):
                print("wallCheck1")
                notWallFlag = False
            if ((x == -2 and y == 1) and (dx == -1 and dy ==  0)):
                notWallFlag = False
            if ((x == -5 and y == 2) and (dx ==  0 and dy == -1)):
                notWallFlag = False
            if ((x == -4 and y == 2) and (dx ==  0 and dy == -1)):
                notWallFlag = False
            # cyan side
            if ((x ==  5 and y == 1) and (dx ==  0 and dy ==  1)):
                notWallFlag = False
            if ((x ==  4 and y == 1) and (dx ==  0 and dy ==  1)):
                notWallFlag = False
            if ((x ==  3 and y == 1) and (dx == -1 and dy ==  0)):
                notWallFlag = False
            if ((x ==  2 and y == 1) and (dx ==  1 and dy ==  0)):
                notWallFlag = False
            if ((x ==  5 and y == 2) and (dx ==  0 and dy == -1)):
                notWallFlag = False
            if ((x ==  4 and y == 2) and (dx ==  0 and dy == -1)):
                notWallFlag = False
        else:
            # magenta side
            if ((x == -6 and y == 1) and (dx ==  0 and dy ==  1)):
                notWallFlag = False
            if ((x == -7 and y == 1) and (dx ==  0 and dy ==  1)):
                notWallFlag = False
            if ((x == -5 and y == 1) and (dx ==  1 and dy ==  0)):
                notWallFlag = False
            if ((x == -4 and y == 1) and (dx == -1 and dy ==  0)):
                notWallFlag = False
            if ((x == -6 and y == 2) and (dx ==  0 and dy == -1)):
                notWallFlag = False
            if ((x == -6 and y == 2) and (dx ==  0 and dy == -1)):
                notWallFlag = False

            # cyan side
            if ((x ==  6 and y == 1) and (dx ==  0 and dy ==  1)):
                notWallFlag = False
            if ((x ==  7 and y == 1) and (dx ==  0 and dy ==  1)):
                notWallFlag = False
            if ((x ==  5 and y == 1) and (dx == -1 and dy ==  0)):
                notWallFlag = False
            if ((x ==  4 and y == 1) and (dx ==  1 and dy ==  0)):
                notWallFlag = False
            if ((x ==  6 and y == 2) and (dx ==  0 and dy == -1)):
                notWallFlag = False
            if ((x ==  6 and y == 2) and (dx ==  0 and dy == -1)):
                notWallFlag = False

        # out of the field
        if (x < FIELDMINX or x > FIELDMAXX or y < FIELDMINY or y > FIELDMAXY):
            notWallFlag = False

        return notWallFlag

    def getNextDirection(self, x, y):
        minStep = self.getField(x, y)
        nextD = Pose2D()
        nextD.x = nextD.y = 0
        for dx, dy in zip([-1, 1, 0, 0], [0, 0, -1, 1]):
            notWallFlag = self.wallCheck(x, y, dx, dy)

            print(x, y, dx, dy, notWallFlag)
            if ((minStep > self.getField(x + dx, y + dy)) and notWallFlag == True):
                minStep = self.getField(x + dx, y + dy)
                nextD.x = dx
                nextD.y = dy
                print("nextDirection", nextD.x, nextD.y, "now: ",self.getField(x, y), "next: ", self.getField(x + nextD.x, y  + nextD.y))
        return nextD

    def makeNextPoint(self, destination):
        # global btrField, btrOdometry, FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
        global FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
        debug = True
        tmpField = self.btrField
        point = self.zoneToXY(destination)
        print("destination is ", destination, point.x, point.y)
        self.setField(point.x, point.y, 1)
        for i in range(FIELDSIZE):
            for x in range(FIELDMINX, FIELDMAXX + 1):
                for y in range(FIELDMINY, FIELDMAXY + 1):
                    if (x == point.x and y == point.y):
                        self.setField(x, y, 1)
                    elif (self.getField(x, y) != MAXSTEP):
                        self.setField(x, y, min(self.getStep(x - 1, y), self.getStep(x, y - 1), \
                                                self.getStep(x + 1, y), self.getStep(x, y + 1)) \
                                            + 1)
                        # wall information
                        if (FIELDMINX == -5):
                            if (x == -5 and y == 1): # M_Z51 = M_Z41 + 1
                                self.setField(x, y, self.getStep(x + 1, y) + 1)
                            if (x == -4 and y == 1): # M_Z41 = M_Z31 + 1
                                self.setField(x, y, self.getStep(x + 1, y) + 1)
                            if (x == -3 and y == 1): # M_Z31 = M_Z32 + 1
                                self.setField(x, y, self.getStep(x, y + 1) + 1)
                            if (x == -2 and y == 1): # M_Z21 <= min(M_Z22, MZ_11) + 1
                                self.setField(x, y, min(self.getStep(x, y + 1), self.getStep(x + 1, y)) + 1)
                            # for cyan side
                            if (x ==  5 and y == 1): # C_Z51 = C_Z41 + 1
                                self.setField(x, y, self.getStep(x - 1, y) + 1)
                            if (x ==  4 and y == 1): # C_Z41 = C_Z31 + 1
                                self.setField(x, y, self.getStep(x - 1, y) + 1)
                            if (x ==  3 and y == 1): # C_Z31 = C_Z32 + 1
                                self.setField(x, y, self.getStep(x, y + 1) + 1)
                            if (x ==  2 and y == 1): # C_Z21 <= min(C_Z22, C_Z11) + 1
                                self.setField(x, y, min(self.getStep(x, y + 1), self.getStep(x - 1, y)) + 1)

                        else:
                            if (x == -6 and y == 1): # M_Z61 = M_Z51 + 1
                                self.setField(x, y, self.getStep(x + 1, y) + 1)
                            if (x == -7 and y == 1): # M_Z71 = M_Z61 + 1
                                self.setField(x, y, self.getStep(x + 1, y) + 1)
                            if (x == -5 and y == 1): # M_Z51 = M_Z52 + 1
                                self.setField(x, y, self.getStep(x, y + 1) + 1)
                            if (x == -4 and y == 1): # M_Z41 <= min(M_Z42, MZ_31) + 1
                                self.setField(x, y, min(self.getStep(x, y + 1), self.getStep(x + 1, y)) + 1)
                            if (x ==  6 and y == 1): # C_Z61 = C_Z51 + 1
                                self.setField(x, y, self.getStep(x - 1, y) + 1)
                            if (x ==  7 and y == 1): # C_Z71 = C_Z61 + 1
                                self.setField(x, y, self.getStep(x - 1, y) + 1)
                            if (x ==  5 and y == 1): # C_Z51 = C_Z52 + 1
                                self.setField(x, y, self.getStep(x, y + 1) + 1)
                            if (x ==  4 and y == 1): # C_Z41 <= min(C_Z42, CZ_31) + 1
                                self.setField(x, y, min(self.getStep(x, y + 1), self.getStep(x - 1, y)) + 1)



        # get optimized route
        if (debug == True):
            for y in range(FIELDMAXY, FIELDMINY  - 1, -1):
                for x in range(FIELDMINX, FIELDMAXX + 1):
                    if (self.getField(x,y) == MAXSTEP):
                        print("*",)
                    else:
                        print(self.getField(x, y),)
                print()

        robotReal = Pose2D()
        robotZone = Pose2D()
        point = Pose2D()
        self.btrRobotino.w_waitOdometry()
        robotReal.x = self.btrOdometry.pose.pose.position.x
        robotReal.y = self.btrOdometry.pose.pose.position.y

        if (robotReal.x > 0):
            robotZone.x = int(robotReal.x) + 1
        else:
            robotZone.x = int(robotReal.x) - 1
        robotZone.y = int(robotReal.y) + 1

        x = int(robotZone.x)
        y = int(robotZone.y)
        # which direction?
        nextD = self.getNextDirection(x, y)
        # where is the turning point?
        point.x = robotZone.x + nextD.x
        point.y = robotZone.y + nextD.y
        print("direction: ",nextD.x, nextD.y)
        for dx, dy, phi in zip([ 1, 0, -1, 0, 0], [0, 1, 0, -1, 0], [0, 90, 180, -90, 360]):
            if (nextD.x == dx and nextD.y == dy):
                theta = phi

        # goToPoint(robotReal.x, robotReal.y, theta) # turn for the next point.
        print(theta)
        self.btrRobotino.w_robotinoTurnAbs(theta) # only turn

        print("direction:", nextD.x, nextD.y)
        while True:
            # print(point)
            if self.getField(point.x, point.y) == 0:
                print("Next Point is goal")
                break
            if (self.getField(point.x + nextD.x, point.y + nextD.y) == self.getField(point.x, point.y) - 1):
                notWallFlag = self.wallCheck(point.x, point.y, nextD.x, nextD.y)
                if (notWallFlag):
                    point.x = point.x + nextD.x
                    point.y = point.y + nextD.y
                else:
                    # there is a wall.
                    break
            else:
                print("Next Point is the turning point")
                break

        # for the nextStep
        nextDD = self.getNextDirection(point.x, point.y)
        theta = -360
        for dx, dy, phi in zip([ 1, 0, -1, 0, 0], [0, 1, 0, -1, 0], [0, 90, 180, -90, 360]): 
            if (nextDD.x == dx and nextDD.y == dy):
                theta = phi

        print("nextPosition: ", point.x, point.y, theta)
        if (point.x == robotZone.x and point.y == robotZone.y):
            theta = 360
        point.theta = theta
        return point

    def getNextPoint(self, pointNumber):
        point = Pose2D()
        route = self.refbox.refboxNavigationRoutes.route
        # zone = route[pointNumber].zone
        zone = route[0].zone

        print("getNextPoint:", zone)
        # if (self.btrOdometry.pose.pose.position.x > 0):
        #     zone = zone - 1000
        print("gazebo zone:", zone)
        point = self.makeNextPoint(zone)
        return point

    def startNavigation(self):
        # global btrField, oldTheta
        # self.initField()
        # print("----")
        # self.setMPStoField()

        print("====")
        self.oldTheta = 90
        while (len(self.refbox.refboxNavigationRoutes.route) == 0 or len(self.refbox.refboxMachineInfo.machines) == 0):
            self.btrRobotino.rate.sleep()
        
        for pointNumber in range(12 + 999):
            print(pointNumber)
            route = self.refbox.refboxNavigationRoutes.route
            print(route)
            if (len(route) == 0):
                print("finished")
            else:
                while True:
                    point = self.getNextPoint(pointNumber)
                    print("point:", point)
                    if (self.navToPoint(point) == True):
                        break
                    print("not arrived?")
                print("arrived #", pointNumber + 1, ": point")
                for i in range(4):
                    self.refbox.sendBeacon()
                    rospy.sleep(2)

    def navToPoint(self, point):
        # global oldTheta
        self.setMPStoField()
    
        self.btrRobotino.w_waitOdometry()
        robot = self.btrOdometry.pose.pose.position

        if (point.x > 0):
            point.x = int(point.x) - 0.5
        else:
            point.x = int(point.x) + 0.5
        point.y = int(point.y) - 0.5
        print("navToPoint ", point.x, point.y, point.theta)
        if (point.theta == 360):
            self.goToPoint(point.x, point.y, oldTheta)
            return True
        else:
            self.goToPoint(point.x, point.y, point.theta)
            self.oldTheta = point.theta
            return False

        print("****")
        # print(self.refbox.refboxNavigationRoutes)
        # print(self.refbox.refboxMachineInfo)

    def goToMPS(self, MPSName, MPSSide):
        Point = FalseValue
        while (Point == FalseValue):
            Point = self.MPS2Point(MPSName, MPSSide)
            print("wait for MPS information:", MPSName, self.MPS2Zone(MPSName), self.MPS2Angle(MPSName))
            self.btrRobotino.rate.sleep()

        self.setMPStoField()
        result = False
        while(result == False):
            print("goToMPS", MPSName, Point)
            print("pointName", self.MPS2Zone(MPSName))
            if (FIELDMINX == -5 and (MPSName == "M-DS" or MPSName == "C-DS")):
                if (self.refbox.teamColor == 1):
                    zone = 31
                else:
                    zone = 1031
                # Point = self.Zone2XYT(Zone, 0, "input")
                point1 = Pose2D()
                point1.theta = 0
            else:
                point1 = self.MPS2Point(MPSName, MPSSide)
                point2 = self.MPS2Point(MPSName, "none")
                point3 = Pose2D()
                point3.x = point2.x + numpy.sign(int(point1.x) - int(point2.x))
                point3.y = point2.y + numpy.sign(int(point1.y) - int(point2.y))
                point3.theta = point1.theta

                print("MPS: ", MPSName)
                print("MPS Side: ", MPSSide)
                print("point1: ", point1.x, point1.y)
                print("point2: ", point2.x, point2.y)
                print("point3: ", point3.x, point3.y)
                zone = abs(point3.x) * 10 + point3.y
                if (point3.x) < 0:
                    zone = zone + 1000
                # Point = self.getNextPoint(point3)

            print("zone: ", int(zone))
            Point = self.makeNextPoint(int(zone))
            # Point.theta = point1.theta
            result = self.navToPoint(Point)
        self.goToPoint(Point.x, Point.y, point1.theta)
        Point.theta = point1.theta
        return zone


    def goToCS(self, command, capColor = 1):
        CS = str(self.refbox.teamColorName) + "-CS" + str(capColor)
        zone = self.goToMPS(CS, "input")
        if (command == CS_OP_RETRIEVE_CAP):
            # get the work from the shelf
            # put the work on the conveyor
            # self.challenge_graspingTest()
            self.getWorkOnShelf()
            self.putWorkOnConveyor()
            # send the command to MPS
            print("CS_OP_RETRIEVE_CAP")
        elif (command == CS_OP_MOUNT_CAP):
            # put the work on the conveyor
            self.putWorkOnConveyor()
            # send the command to MPS
            print("CS_OP_MOUNT_CAP")
        self.prepareMachine.machine = CS    # "C-CS1"
        self.prepareMachine.cs_operation = command
        self.prepareMachine.wait = True
        self.refbox.sendPrepareMachine(self.prepareMachine)
        self.goToZone(zone)
        return CS

    def goToBS(self, baseColor, getSide):
        BS = str(self.refbox.teamColorName) + "-BS"
        zone = self.goToMPS(BS, getSide)
        self.prepareMachine.machine = BS
        if (getSide == "input"):
            getSideNo = 1
        else:
            getSideNo = 2
        self.prepareMachine.bs_side = getSideNo
        self.prepareMachine.bs_base_color = baseColor
        self.prepareMachine.wait = True
        self.refbox.sendPrepareMachine(self.prepareMachine)
        # self.goToZone(zone)
        return zone

    def deliveryStation(self, orderInfo):
        DS = str(self.refbox.teamColorName) + "-DS"
        self.prepareMachine.machine = DS
        self.prepareMachine.ds_order_id = orderInfo
        self.prepareMachine.wait = True
        self.refbox.sendPrepareMachine(self.prepareMachine)

    def goToZone(self, zone):
        print("goToZone: ", zone)
        Point = self.zoneToXY(zone)
        if (Point.x > 0):
            Point.x -= 0.5
        else:
            Point.x += 0.5
        Point.y -= 0.5
        print("goToPoint: ", Point.x, Point.y, Point.theta)
        self.goToPoint(Point.x, Point.y, Point.theta)

    def startProduction(self):
        # global oldTheta, btrField
        debug = False
        # self.btrRobotino.w_goToInputVelt()
        # self.putWorkOnConveyor()


        while (self.refbox.refboxOrderInfoFlag == False):
            print("wait OrderInfo")
            self.btrRobotino.rate.sleep()

        orderC0 = [i for i in self.refbox.refboxOrderInfo.orders if i.complexity == 0]
        orderC1 = [i for i in self.refbox.refboxOrderInfo.orders if i.complexity == 1]

        order = orderC0 + orderC1

        nowTime = self.refbox.refboxGameTime.sec
        print("time: ", nowTime)
        print("orders: ", order)
        nowOrders = [i for i in order if nowTime < i.delivery_period_end]
        orders= sorted(nowOrders, key=lambda nowOrders: nowOrders.delivery_period_begin)
        print("nowOrders" , nowOrders)
        if (len(orders) > 0):
            print("target:", len(orders), orders[0])
            orderInfo = orders[0]
            # for challenge, cap and base Color is fixed as 1.
            if (FIELDMINX == -5):
                orderInfo.cap_color = 1
                orderInfo.base_color = 1
            navPoint = Pose2D()
            if self.refbox.teamColor == 1:
                navPoint.x = 3
                navPoint.y = 2
            else:
                navPoint.x = -3
                navPoint.y = 2
            navPoint.theta = 90
            print("BTR-1")
            print(navPoint)
            tmpPoint = Pose2D()
            tmpPoint.x = navPoint.x
            tmpPoint.y = navPoint.y
            tmpPoint.theta = navPoint.theta
            self.navToPoint(navPoint)
            print("BTR-2")

            if (debug == True):
                navPoint = tmpPoint
                navPoint.x += 1
                self.navToPoint(navPoint)


            # go to Cap Station to retrieve the cap.
            capColor = orderInfo.cap_color
            baseColor = orderInfo.base_color
            ### code for debug
            if (debug != True):
                CS = self.goToCS(CS_OP_RETRIEVE_CAP, capColor)
                # get the base and put it at the slide of RS1.
                # turn: we need the object information to turn clockwise/ counter clockwise.
                # go to the output side
                zone = self.goToMPS(CS, "output")
                ##self.getWork(CS)
                self.getWorkOnConveyor()
                ##RS = self.goToRS(SLIDE)
                self.goToZone(zone)
                RS = str(self.refbox.teamColorName) + "-RS" + str(1)
                zone = self.goToMPS(RS, "input")
                # putWorkOnSlide()
                self.putWorkOnSlide()
                self.goToZone(zone)
                # go to BS
                zone = self.goToBS(baseColor, "output")
                # get the base at BS.
                self.getWorkOnConveyor()    # this is not adjusted for input side.
                # put the base at CS in order to mount the cap.
                ##self.GoToCS(MOUNT, CS)
                self.goToZone(zone)
                CS = self.goToCS(CS_OP_MOUNT_CAP, capColor)
                # get the product and put it at the DS.
                zone = self.goToMPS(CS, "output")
                ##self.getWork(CS)
                self.getWorkOnConveyor()
                self.goToZone(zone)
            ##self.goToDS(ORDER)
            DS = str(self.refbox.teamColorName) + "-DS"
            zone = self.goToMPS(DS, "input")
            # self.putWorkOnConveyore()
            if (FIELDMINX == -5):
                self.btrRobotino.w_putWork()
            else:
                self.putWorkOnConveyor()
            self.deliveryStation(orderInfo.id)

# main
#
if __name__ == '__main__':

    robotNum = 1
    gazeboFlag = True

    nodeName = "btr_2024_" + str(robotNum)
    rospy.init_node(nodeName)
    rate = rospy.Rate(10)

    refbox = btr_refbox.refbox(teamName = "BabyTigers-R", robotNum = robotNum, gazeboFlag = gazeboFlag)
    rcll2024 = btr_rcll(teamName = "BabyTigers-R", robotNum = robotNum, gazeboFlag = gazeboFlag, refbox = refbox)

    rcll2024.challenge("findMPS")

    refbox.sendBeacon()
    rate.sleep()


