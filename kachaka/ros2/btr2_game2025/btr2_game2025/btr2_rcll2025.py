#!/usr/bin/python
import sys
import subprocess
import rclpy
import btr2_rcll2025
import btr2_refbox
import kachaka_api
import os
import math

from geometry_msgs.msg import Pose, Pose2D, PoseStamped, PointStamped, Point, Vector3
from refbox_msgs.msg import BeaconSignal, ExplorationInfo, \
                            ExplorationSignal, ExplorationZone, GameState, \
                            LightSpec, MachineInfo, Machine, \
                            MachineReportEntry, MachineReportEntryBTR, \
                            MachineReportInfo, OrderInfo, Order, \
                            ProductColor, RingInfo, Ring, Team, Time, \
                            NavigationRoutes, Route
from refbox_msgs.srv import SendBeaconSignal, SendMachineReport, \
                            SendMachineReportBTR, SendPrepareMachine

TEAMNAME = "BabyTigers-R"

### for Challenge Track
FIELDMINX = -5
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


class btr2_rcll(object):
    def __init__(self, teamName = "Babytigers-R", robotNum = 0, gazeboFlag = False, refbox = None):
        global FIELDMINX, FIELDMINY
        self.topicName = ""
        self.gazeboFlag = gazeboFlag
        self.robotNum = robotNum
        if (gazeboFlag):
            self.topicName = "/btr-robot" + str(robotNum)
        # else:
        #     self.pg = module_photographer()
        #     self.bd = module_belt_detect()
        #     self.c0d = module_c0_detect()

        if (refbox == None):
            print("please set refbox arg")
            # break
            exit()
        self.refbox = refbox

        self.btrField = [[0 for x in range(FIELDMINX, FIELDMAXX + 1)] for y in range(FIELDMINY, FIELDMAXY + 1)]
        self.machineReport = MachineReportEntryBTR()
        self.prepareMachine = SendPrepareMachine.Request()

        self.kachakaIP = os.getenv('kachaka_IP')
        self.kachaka = kachaka_api.KachakaApiClient(target=self.kachakaIP + ":26400")
        self.kachaka.set_auto_homing_enabled(False)
        self.kachaka.get_battery_info()


    def initField(self):
        self.btrField = [[0 for x in range(FIELDSIZEX)] for y in range(FIELDSIZEY)]
        for zone in range(2):
            for x in range(1, 8):
                for y in range(1, 8):
                    zoneName = str(zone * 100 + x) + str(y)
                    zoneX[zoneName] = (x - 0.5) * (-zone * 2 + 1)
                    zoneY[zoneName] = y - 0.5

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



    def exploration(self):
        print("exploration challenge")

    def production(self):
        print("production challenge")
        debug = False
        
        while (self.refbox.refboxOrderInfoFlag == False):
            print("wait OrderInfo")
            self.refbox.sendBeacon()

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
                self.releaseWork()
            else:
                self.putWorkOnConveyor()
            self.deliveryStation(orderInfo.id)

    def MPS2Zone(self, MPSName):
        while (self.refbox.refboxMachineInfoFlag == False):
            self.refbox.sendBeacon()

        for i in self.refbox.refboxMachineInfo.machines:
            # print("MPS2Zone: ", MPSName, i.name, MPSName == i.name, i.zone)
            if (i.name == MPSName):
                return i.zone
        return False

    def MPS2Angle(self, MPSName):
        while (self.refbox.refboxMachineInfoFlag == False):
            self.refbox.sendBeacon()

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
    
    def grasping(self):
        # todo
        print("grasping challenge")

    def getWorkOnConveyor(self):
        # todo
        print("getWorkOnConveyor")

    def releaseWork(self):
        # todo
        print("releaseWork")

    def putWorkOnSlide(self):
        # todo
        print("putWorkOnSlide")

    def putWorkOnConveyore(self):
        #todo
        print("putWorkOnConveyore")

    def navigation(self):
        print("navigation challenge")

        ### route情報を受取る
        print("====")
        self.oldTheta = 90
        while (len(self.refbox.refboxNavigationRoutes.route) == 0 or \
               len(self.refbox.refboxMachineInfo.machines) == 0):
            rclpy.spin_once(self.refbox)

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
                self.rclpy.spin_once(refbox)

    def main_exploration(self):
        print("main challege: exploration")

    def main_production(self):
        # to do
        print("main challenge: production")

    def getNextPoint(self, pointNumber):
        point = Pose2D()
        route = self.refbox.refboxNavigationRoutes.route
        # zone = route[pointNumber].zone
        zone = route[0].zone
            
        print("getNextPoint:", zone)
        # if (self.btrOdometry.pose.pose.position.x > 0):
        #     zone = zone - 1000
        print("gazebo zone:", zone)
        # point = self.makeNextPoint(zone)
        point = self.zoneToXY(zone)
        return point

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
            print("Debug: ")
            for y in range(FIELDMAXY, FIELDMINY  - 1, -1):
                for x in range(FIELDMINX, FIELDMAXX + 1):
                    if (self.getField(x,y) == MAXSTEP):
                        print("*",)
                    else:
                        print(self.getField(x, y), end = ', ')
                print()

        robotReal = Pose2D()
        robotZone = Pose2D()
        point = Pose2D()
        # self.btrRobotino.w_waitOdometry()
        self.refbox.robotOdometryFlag = False
        while self.refbox.robotOdometryFlag == False:
            rclpy.spin_once(self.refbox)
        robotReal.x = self.refbox.robotOdometry.pose.pose.position.x
        robotReal.y = self.refbox.robotOdometry.pose.pose.position.y

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

    def navToPoint(self, point):
        self.kachaka_speak("今から"+str(point.x)+"の"+str(point.y)+"の座標へ移動するね")
        
        if (point.x > 0):
            point.x = int(point.x) - 0.5
        else:
            point.x = int(point.x) + 0.5
        point.y = int(point.y) - 0.5

        self.kachaka_move_to_pose(point.x, point.y, point.theta)
        self.kachaka_speak("移動したよ")

    def challenge(self, name):
        ### globals()[name]()  # 危険: 任意の関数が実行できてしまう

        # 関数名に対応する関数の辞書
        challenge_functions = {
            "exploration": self.exploration,
            "production": self.production,
            "grasping": self.grasping,
            "navigation": self.navigation,
            "main_exploration": self.main_exploration,
            "main_production": self.main_production
        }

        # set for the position
        ## setting for Challenge Track
        pose = Pose2D()
        pose.x = -1.0 * self.robotNum - 1.5
        pose.y = 0.5
        pose.theta = 90
        self.kachakaStartPosition = True
        if (name == "grasping"):
            startX =     [ -0.5, -4.5, -0.5]
            startY =     [  0.5,  1.5,  4.5]
            startTheta = [   90,   90,  180]
            pose.x = startX[self.robotNum - 1]
            pose.y = startY[self.robotNum - 1]
            pose.theta = startTheta[self.robotNum - 1]
            self.kachakaStartPosition = False
        elif (name == "main_exploration" or name == "main_production"):
            pose.x = 3.5 + self.robotNum
        print("Team Color: ", self.refbox.teamColor)
        if (self.refbox.teamColor == 1):
            pose.x = -pose.x
        print(pose.x, pose.y, pose.theta)
        # self.btrRobotino.w_resetOdometry(pose)
        pose.theta = pose.theta / 180 * math.pi
        self.kachaka_set_robot_pose(pose)
        # self.kachaka.set_robot_pose({ "x": pose.x, "y": pose.y, "theta": pose.theta })
        self.refbox.sendBeacon()
        print(self.kachaka.get_robot_pose())
        self.kachaka_speak(name + "を頑張るよ．")
        print(0/0)
        self.kachaka_move_to_pose(pose.x, pose.y + 1.0, pose.theta)

        # 該当する関数があれば実行
        if name in challenge_functions:
            print(f"[challenge] {name} challenge start.")
            # challenge_functions[name]()
        else:
            print(f"[challenge] Unknown challenge: {name}")

    def kachaka_move_status(self, pose1):
        precision = 10
        self.refbox.sendBeacon()
        pose2 = self.kachaka_get_robot_pose()

        result = True
        if (int(pose1.x * precision) != int(pose2.x * precision)):
            result = False
        if (int(pose1.y * precision) != int(pose2.y * precision)):
            result = False
        if (int(pose1.theta * precision) != int(pose2.theta * precision)):
            result = False
        # print(f"[kachaka_move_status] result: {result}")
        return result

    def kachaka_speak(self, data):
        kachaka_command = f"export kachaka_IP={self.kachakaIP}; python3 btr2_kachaka.py speak {data} > /dev/null 2>&1 &"
        self.refbox.get_logger().info(kachaka_command)
        os.system(kachaka_command)

    def kachaka_move_to_pose(self, x, y, theta):
        self.refbox.get_logger().info(f"[kachaka_move_to_pose in the field]: ({x}, {y}, {theta})")

        kachaka_x =  y + 0.5
        kachaka_y = -x + 4.5
        kachaka_theta = theta - 3.14159/2.0
        print(f"[kachaka_move_to_pose in kachaka]: ({kachaka_x}, {kachaka_y}, {kachaka_theta})")
        kachaka_command = f"export kachaka_IP={self.kachakaIP}; python3 btr2_kachaka.py move_to_pose {kachaka_x}  {kachaka_y} {kachaka_theta} > /dev/null 2>&1 &"
        self.refbox.get_logger().info(kachaka_command)
        os.system(kachaka_command)
        print(f"[kachaka_move_to_pose] wait for move")

        pose = self.kachaka_get_robot_pose()
        while (self.kachaka_move_status(pose)):  # 動き出すのを待つ
            self.refbox.sendBeacon()
            # print(f"[kachaka_move_to_pose]: ", self.kachaka_move_status(pose))
        pose.x = kachaka_x
        pose.y = kachaka_y
        pose.theta = kachaka_theta
        while (self.kachaka_move_status(pose) == False):    # 目的地に着くのを待つ
            print(f"[kachaka_move_to_pose]: ", self.kachaka.get_running_command(), self.kachaka.is_command_running())
            self.refbox.sendBeacon()

    def kachaka_ros_odometry(self):
        odometry = self.kachaka.get_ros_odometry()
        # print(f"[kachaka_ros_odometry] odometry: {odometry}")
        return odometry

    def kachaka_get_robot_pose(self):
        pose = self.kachaka.get_robot_pose()
        # print(f"[kachaka_get_robot_pose] pose: {pose}")
        return pose

    def kachaka_set_robot_pose(self, pose):
        rcll = pose
        rcll.x =  pose.y
        rcll.y = -pose.x

        kachaka_command = f"export kachaka_IP={self.kachakaIP}; python3 btr2_kachaka.py set_robot_pose {rcll.x} {rcll.y} {rcll.theta}> /dev/null 2>&1 &"
        self.refbox.get_logger().info(kachaka_command)
        os.system(kachaka_command)
        self.kachaka.set_robot_pose({ "x": rcll.x, "y": rcll.y, "theta": rcll.theta })


def main(args=None):
    rclpy.init(args=args)
    args = sys.argv
    topicName = ""
    gazeboFlag = True
    robotNum = 1

    ### set challenge information by args
    if (len(args) >= 2):
        challenge = args[1]
        if (len(args) >= 3):
            robotNum = int(args[2])
        if (gazeboFlag == True):
            topicName = "/robotino" + str(robotNum)

    ### set up for RefBox
    refbox = btr2_refbox.refbox(teamName = "Babytigers-R", robotNum = robotNum, gazeboFlag = False)
    refbox.sendBeacon()

    print(refbox)
    rcll2025 = btr2_rcll(teamName = "Babytigers-R", robotNum = robotNum, gazeboFlag = gazeboFlag, refbox = refbox)

    self.kachaka_speak("こんにちは、btr2_rcll2025.py を実行中です．")

    while True:
        while rclpy.ok():
            rclpy.spin_once(refbox)
            ### check for receiving GameState information
            if (refbox.refboxGameStateFlag == True):
                print("game2025:", challenge, refbox.refboxGameState)
                if (challengeFlag):
                    ### Checking for changing of GmaePhase
                    if (oldGamePhase != refbox.refboxGamePhase):
                        print("refboxGamePhase: ", refbox.refboxGamePhase)
                        oldGamePhase = refbox.refboxGamePhase
                    
                    ### For Challenge Track
                    
                    #### Exploration Challenge
                    if (challenge == "exploration" and refbox.refboxGamePhase == 20 ):
                        challenge("exploration")
                        challengeFlag = False

# main
#
if __name__ == '__main__':
    main()

