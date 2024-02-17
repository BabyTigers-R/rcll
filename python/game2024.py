#!/usr/bin/python
import struct
import time
import math
import sys
import rospy
import numpy
import cv2
# import robotino2022
import btr2024
import btr_refbox

# from module_photographer import module_photographer
# from module_work_detect import module_work_detect
# from module_line_detect import module_line_detect
from module_photographer import module_photographer
# from module_belt_detect import module_belt_detect
from module_photographer_by_c920 import module_photographer_by_c920
from module_belt_detect_for_c920 import module_belt_detect_for_c920
from module_center_of_gravity_detect import module_center_of_gravity_detect

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

TEAMNAME = "BabyTigers-R"

# FIELDMINX = -5
# FIELDMAXX = -1
# FIELDMINY =  1
# FIELDMAXY =  5
FIELDMINX = -7
FIELDMAXX = 7
FIELDMINY = 1
FIELDMAXY = 8
FIELDSIZEX = (FIELDMAXX - FIELDMINX) + 1
FIELDSIZEY = (FIELDMAXY - FIELDMINY) + 1
FIELDSIZE = FIELDSIZEX * FIELDSIZEY
MAXSTEP = 999

zoneX = { "S11" : -0.5, "S21" : -1.5, "S31" : -2.5, "S41" : -3.5, "S51" : -4.5,
          "S12" : -0.5, "S22" : -1.5, "S32" : -2.5, "S42" : -3.5, "S52" : -4.5,
          "S13" : -0.5, "S23" : -1.5, "S33" : -2.5, "S43" : -3.5, "S53" : -4.5,
          "S14" : -0.5, "S24" : -1.5, "S34" : -2.5, "S44" : -3.5, "S54" : -4.5,
          "S15" : -0.5, "S25" : -1.5, "S35" : -2.5, "S45" : -3.5, "S55" : -4.5 }
zoneY = { "S11" :  0.5, "S21" :  0.5, "S31" :  0.5, "S41" :  0.5, "S51" :  0.5,
          "S12" :  1.5, "S22" :  1.5, "S32" :  1.5, "S42" :  1.5, "S52" :  1.5,
          "S13" :  2.5, "S23" :  2.5, "S33" :  2.5, "S43" :  2.5, "S53" :  2.5,
          "S14" :  3.5, "S24" :  3.5, "S34" :  3.5, "S44" :  3.5, "S54" :  3.5,
          "S15" :  4.5, "S25" :  4.5, "S35" :  4.5, "S45" :  4.5, "S55" :  4.5 }
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

def robotinoOdometry(data):
    global btrOdometry, btrBeaconCounter
    btrOdometry = data
    ##btrOdometry.pose.pose.position.z = quat.z / math.pi * 180
    btrOdometry.pose.pose.position.z = btrOdometry.pose.pose.position.z # / math.pi * 180

def w_findMPS():
    btrRobotino.w_getMPSLocation()
    if (btrRobotino.MPS_find == True and btrRobotino.MPS_id > 0):
        print(btrRobotino.MPS_id)
        if not (btrRobotino.MPS_id in machineName):
            print(btrRobotino.MPS_id, "is not ID?")
            return False
        name = machineName[btrRobotino.MPS_id]
        zone = btrRobotino.MPS_zone
        machineReport.name = name[0: len(name) - 2]
        if (name[4 : 5] == "-"):
            machineReport.type = name[2 : 4]
        else:
            machineReport.type = name[2 : 5]
        zone = int(btrRobotino.MPS_zone[3 : 5])
        if (btrRobotino.MPS_zone[0: 1] == "M"):
            zone = -zone
        machineReport.zone = zone
        machineReport.rotation = btrRobotino.MPS_phi
        refbox.sendMachineReport(machineReport)

        btrRobotino.w_addMPS(name, zone, btrRobotino.MPS_phi)
    return btrRobotino.MPS_find

def goToPoint(x, y, phi):
    btrRobotino.w_robotinoMove(0, 0)
    btrRobotino.w_waitOdometry()
    nowX = btrOdometry.pose.pose.position.x
    nowY = btrOdometry.pose.pose.position.y
    nowPhi = btrOdometry.pose.pose.position.z
    dist = ((x - nowX)**2 + (y - nowY)**2) **0.5
    # print(dist)
    if (dist > 0.30):
      turn = numpy.rad2deg(numpy.arctan2(y - nowY, x - nowX))
      print(nowX, nowY, nowPhi, "=>", x, y, phi)
      # print(turn, turn - nowPhi)
      btrRobotino.w_robotinoTurn(turn - nowPhi)
      btrRobotino.w_robotinoMove(dist, 0)
    else:
      print("dist <= 0.30")
      moveX = x - nowX
      moveY = y - nowY
      # print(moveX, moveY, nowPhi)
      rad = math.radians(nowPhi)
      distX = moveX * math.cos(-rad) - moveY * math.sin(-rad)
      distY = moveX * math.sin(-rad) + moveY * math.cos(-rad)
      btrRobotino.w_robotinoMove(distX, distY)
    nowPhi = btrOdometry.pose.pose.position.z
    # print(phi, phi - nowPhi)
    btrRobotino.w_robotinoTurn(phi - nowPhi)
    # btrRobotino.w_robotinoMove(x, y)
    # btrRobotino.w_robotinoTurn(phi)
# 
# challenge program
#

def graspTransparent():
    name = "ref_img"
    pg1 = module_photographer(name)
    pg2 = module_photographer_by_c920(name)

    btrRobotino.w_goToWall(0.9)
    btrRobotino.w_goToMPSCenter()

    #btrRobotino.w_goToInputVelt()
    btrRobotino.w_parallelMPS()
    btrRobotino.w_goToWall(0.4)

    adjustment(name, pg2, False)
    btrRobotino.w_goToWall(0.175)

    btrRobotino.w_robotinoMove(0, -0.2)
    #btrRobotino.w_robotinoMove(5, 0)

    btrRobotino.w_pick_rs()
    btrRobotino.w_looking_for_C0()
    a_previous = 0
    for i in range(10):
        pg1.run()
        rospy.sleep(1)

        d = module_center_of_gravity_detect(name)
        ato_take = d.run()
        d.show_result()
        if ato_take == -1: # left
            btrRobotino.w_robotinoMove(0, -0.02)
        elif ato_take == 1: # right
            btrRobotino.w_robotinoMove(0, 0.02)
        elif ato_take == 2: # none detected
            btrRobotino.w_robotinoMove(0, -a_previous * 0.02)
        else:
            break
        a_previous = int((-ato_take)*(ato_take % 2))

    btrRobotino.w_put_rs()

    btrRobotino.w_move_g_C0()
    btrRobotino.w_goToWall(0.9)
    btrRobotino.w_goToMPSCenter()
    #btrRobotino.w_goToMPSCenterLRF()
    #btrRobotino.w_goToInputVelt()
    btrRobotino.w_parallelMPS()
    btrRobotino.w_goToWall(0.4)

    adjustment(name, pg2, False)
    btrRobotino.w_goToWall(0.17)
    btrRobotino.w_putWork()

    btrRobotino.w_goToWall(0.9)


def startGrasping():
    name = "ref_img"
    pg = module_photographer()
    for _ in range(3):
        print("{} / 3 repeation".format(_+1))
        challengeFlag = False

        btrRobotino.w_goToOutputVelt()
        btrRobotino.w_goToWall(0.35)
        btrRobotino.w_parallelMPS()
        btrRobotino.w_goToWall(0.17)

        adjustment(name, pg, True)
        btrRobotino.w_getWork()

        if (robotNum != 2):
            btrRobotino.w_turnClockwise()
        else:
            btrRobotino.w_turnCounterClockwise()

        btrRobotino.w_goToInputVelt()
        btrRobotino.w_goToWall(0.35)
        btrRobotino.w_parallelMPS()
        btrRobotino.w_goToWall(0.17)

        adjustment(name, pg, True)

        btrRobotino.w_putWork()
        if (robotNum != 2):
            btrRobotino.w_turnCounterClockwise()
        else:
            btrRobotino.w_turnClockwise()

def adjustment(name, pg, rs):
    a_previous = 0
    for i in range(10):
        pg.run()
        rospy.sleep(1)
        if rs:
            d = module_belt_detect(name)
        else:
            d = module_belt_detect_for_c920(name)
        ato_take = d.run()
        d.show_result()
        if ato_take == -1: # left
            btrRobotino.w_robotinoMove(0, -0.015)
        elif ato_take == 1: # right
            btrRobotino.w_robotinoMove(0, 0.015)
        elif ato_take == 2: # none detected
            btrRobotino.w_robotinoMove(0, -a_previous * 0.015)
        else:
            break
        a_previous = int((-ato_take)*(ato_take % 2))

def startGrasping_by_c920():
    name = "ref_img"
    pg = module_photographer_by_c920(name)
    for _ in range(3):
        print("{} / 3 repeation".format(_+1))
        challengeFlag = False

        btrRobotino.w_goToWall(0.90)
        btrRobotino.w_goToOutputVelt()
        btrRobotino.w_parallelMPS()
        btrRobotino.w_goToWall(0.40)

        adjustment(name, pg, False)

        btrRobotino.w_goToWall(0.17)

        #btrRobotino.w_bringWork()
        btrRobotino.w_getWork()
        if (robotNum != 2):
            btrRobotino.w_turnClockwise()
        else:
            btrRobotino.w_turnCounterClockwise()

        btrRobotino.w_goToWall(0.90)
        btrRobotino.w_goToInputVelt()
        btrRobotino.w_parallelMPS()
        btrRobotino.w_goToWall(0.40)

        adjustment(name, pg, False)

        btrRobotino.w_goToWall(0.17)

        btrRobotino.w_putWork()
        if (robotNum != 2):
            btrRobotino.w_turnCounterClockwise()
        else:
            btrRobotino.w_turnClockwise()

def initField():
    global btrField
    # btrField = [[0 for y in range(FIELDSIZEY)] for x in range(FIELDSIZEX)]
    btrField = [[0 for x in range(FIELDSIZEX)] for y in range(FIELDSIZEY)]
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

def setField(x, y, number):
    global FIELDMINX, FIELDMINY
    btrField[y - FIELDMINY][x - FIELDMINX] = number

def getField(x, y):
    global FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
    if (int(x) < FIELDMINX or FIELDMAXX < int(x) or int(y) < FIELDMINY or FIELDMAXY < int(y)):
        # print("getField range over: ", x, y)
        return MAXSTEP 
    return btrField[int(y) - FIELDMINY][int(x) - FIELDMINX]

def zoneToPose2D(zone):
    point = Pose2D()
    if zone < 0:        
        zone = zone + 255
    point.y = zone % 10
    point.x = (zone % 100) // 10
    if (zone > 100):
        point.x = -point.x
    return point

def setMPStoField():
    global btrField
    point = Pose2D()
     
    if (len(refbox.refboxMachineInfo.machines) > 0):
        btrRobotino.machineList = ""
        for machine in refbox.refboxMachineInfo.machines:
            btrRobotino.w_addMPS(machine.name, machine.zone)

    print(btrRobotino.machineList)
    for machine in btrRobotino.machineList:
        print(machine)
        point = zoneToPose2D(machine[1])
        print("setMPS: ", machine[0], machine[1], point.x, point.y)
        if (point.x == 0 and point.y == 0):
            print("received NULL data for MPS", machine.name)
        else:
            setField(point.x, point.y, MAXSTEP)

def getStep(x, y):
    global FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
    if ((x < FIELDMINX or FIELDMAXX < x) or (y < FIELDMINY or FIELDMAXY < y)):
        return MAXSTEP

    step = getField(x, y)
    if (step == 0):
        step = MAXSTEP
    return step

def wallCheck(x, y, dx, dy):
    notWallFlag = True
    # magenta side
    if (FIELDMINX == -5):
        if ((x == -5 and y == 1) and (dx ==  0 and dy ==  1)):
            notWallFlag = False
        if ((x == -4 and y == 1) and (dx ==  0 and dy ==  1)):
            notWallFlag = False
        if ((x == -3 and y == 1) and (dx ==  1 and dy ==  0)):
            notWallFlag = False
        if ((x == -2 and y == 1) and (dx == -1 and dy ==  0)):
            notWallFlag = False
        if ((x == -5 and y == 2) and (dx ==  0 and dy == -1)):
            notWallFlag = False
        if ((x == -4 and y == 2) and (dx ==  0 and dy == -1)):
            notWallFlag = False
    else:
        # amgenta side
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

    return notWallFlag

def getNextDirection(x, y):
    minStep = getField(x, y)
    nextD = Pose2D()
    nextD.x = nextD.y = 0
    for dx, dy in zip([-1, 1, 0, 0], [0, 0, -1, 1]):
        notWallFlag = wallCheck(x, y, dx, dy)

        if ((minStep > getField(x + dx, y + dy)) and notWallFlag):
            minStep = getField(x + dx, y + dy)
            nextD.x = dx
            nextD.y = dy
    print("nextDirection", dx, dy, "now: ",getField(x, y), "next: ", getField(x +dx, y +dy))
    return nextD

def makeNextPoint(destination):
    global btrField, btrOdometry, FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
    tmpField = btrField
    point = zoneToPose2D(destination)
    print("destination is ", destination, point.x, point.y)
    setField(point.x, point.y, 1)
    for i in range(FIELDSIZE):
        for x in range(FIELDMINX, FIELDMAXX + 1):
            for y in range(FIELDMINY, FIELDMAXY + 1):
                if (x == point.x and y == point.y):
                    setField(x, y, 1)
                elif (getField(x, y) != MAXSTEP):
                    setField(x, y, min(getStep(x - 1, y), getStep(x, y - 1), \
                                       getStep(x + 1, y), getStep(x, y + 1)) \
                                   + 1)
                    # wall information
                    if (FIELDMINX == -5):
                        if (x == -5 and y == 1): # M_Z51 = M_Z41 + 1
                            setField(x, y, getStep(x + 1, y) + 1)
                        if (x == -4 and y == 1): # M_Z41 = M_Z31 + 1
                            setField(x, y, getStep(x + 1, y) + 1)
                        if (x == -3 and y == 1): # M_Z31 = M_Z32 + 1
                            setField(x, y, getStep(x, y + 1) + 1)
                        if (x == -2 and y == 1): # M_Z21 <= min(M_Z22, MZ_11) + 1
                            setField(x, y, min(getStep(x, y + 1), getStep(x + 1, y)) + 1)
                    else:
                        if (x == -6 and y == 1): # M_Z61 = M_Z51 + 1
                            setField(x, y, getStep(x + 1, y) + 1)
                        if (x == -7 and y == 1): # M_Z71 = M_Z61 + 1
                            setField(x, y, getStep(x + 1, y) + 1)
                        if (x == -5 and y == 1): # M_Z51 = M_Z52 + 1
                            setField(x, y, getStep(x, y + 1) + 1)
                        if (x == -4 and y == 1): # M_Z41 <= min(M_Z42, MZ_31) + 1
                            setField(x, y, min(getStep(x, y + 1), getStep(x + 1, y)) + 1)
                        if (x ==  6 and y == 1): # C_Z61 = C_Z51 + 1
                            setField(x, y, getStep(x - 1, y) + 1)
                        if (x ==  7 and y == 1): # C_Z71 = C_Z61 + 1
                            setField(x, y, getStep(x - 1, y) + 1)
                        if (x ==  5 and y == 1): # C_Z51 = C_Z52 + 1
                            setField(x, y, getStep(x, y + 1) + 1)
                        if (x ==  4 and y == 1): # C_Z41 <= min(C_Z42, CZ_31) + 1
                            setField(x, y, min(getStep(x, y + 1), getStep(x - 1, y)) + 1)



    # get optimized route
    if (False):
        for y in range(FIELDMAXY, FIELDMINY  - 1, -1):
            for x in range(FIELDMINX, FIELDMAXX + 1):
                if (getField(x,y) == MAXSTEP):
                    print("*",)
                else:
                    print(getField(x, y),)
            print()

    robotReal = Pose2D()
    robotZone = Pose2D()
    point = Pose2D()
    btrRobotino.w_waitOdometry()
    robotReal.x = btrOdometry.pose.pose.position.x
    robotReal.y = btrOdometry.pose.pose.position.y

    if (robotReal.x > 0):
        robotZone.x = int(robotReal.x) + 1
    else:
        robotZone.x = int(robotReal.x) - 1
    robotZone.y = int(robotReal.y) + 1

    x = int(robotZone.x)
    y = int(robotZone.y)
    # which direction?
    nextD = getNextDirection(x, y)
    # where is the turning point?
    point.x = robotZone.x + nextD.x
    point.y = robotZone.y + nextD.y
    # print("direction: ",nextD.x, nextD.y)
    for dx, dy, phi in zip([ 1, 0, -1, 0, 0], [0, 1, 0, -1, 0], [0, 90, 180, -90, 360]):
        if (nextD.x == dx and nextD.y == dy):
            theta = phi

    # goToPoint(robotReal.x, robotReal.y, theta) # turn for the next point.
    btrRobotino.w_robotinoTurnAbs(theta) # only turn

    print("direction:", nextD.x, nextD.y)
    while True:
        # print(point)
        if getField(point.x, point.y) == 0:
            print("Next Point is goal")
            break
        if (getField(point.x + nextD.x, point.y + nextD.y) == getField(point.x, point.y) - 1):
            notWallFlag = wallCheck(point.x, point.y, nextD.x, nextD.y)
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
    nextDD = getNextDirection(point.x, point.y)
    theta = -360
    for dx, dy, phi in zip([ 1, 0, -1, 0, 0], [0, 1, 0, -1, 0], [0, 90, 180, -90, 360]): 
        if (nextDD.x == dx and nextDD.y == dy):
           theta = phi

    print("nextPosition: ", point.x, point.y, theta)
    if (point.x == robotZone.x and point.y == robotZone.y):
        theta = 360
    point.theta = theta
    return point

def getNextPoint(pointNumber):
    point = Pose2D()
    route = refbox.refboxNavigationRoutes.route
    # zone = route[pointNumber].zone
    zone = route[0].zone

    print("getNextPoint:", zone)
    if (btrOdometry.pose.pose.position.x > 0):
        zone = zone - 1000
    print("gazebo zone:", zone)
    point = makeNextPoint(zone)
    return point

def startNavigation():
    global btrField, oldTheta
    # initField()
    # print("----")
    # setMPStoField()
    print("====")
    oldTheta = 90
    while (len(refbox.refboxNavigationRoutes.route) == 0):
        btrRobotino.rate.sleep()
        
    for pointNumber in range(12 + 999):
        print(pointNumber)
        route = refbox.refboxNavigationRoutes.route
        print(route)
        if (len(route) == 0):
            print("finished")
        else:
            while True:
                point = getNextPoint(pointNumber)
                print("point:", point)
                if (navToPoint(point) == True):
                    break
                print("not arrived?")
            print("arrived #", pointNumber + 1, ": point")
            for i in range(4):
                refbox.sendBeacon()
                rospy.sleep(2)

def navToPoint(point):
    global oldTheta
    setMPStoField()
    
    btrRobotino.w_waitOdometry()
    robot = btrOdometry.pose.pose.position

    if (point.x > 0):
        point.x = point.x * 1.0 - 0.5
    else:
        point.x = point.x * 1.0 + 0.5
    point.y = point.y * 1.0 - 0.5
    print("navToPoint ", point.x, point.y, point.theta)
    if (point.theta == 360):
        goToPoint(point.x, point.y, oldTheta)
        return True
    else:
        goToPoint(point.x, point.y, point.theta)
        oldTheta = point.theta
        return False

    print("****")
    # print(refbox.refboxNavigationRoutes)
    # print(refbox.refboxMachineInfo)

def startProductionC0():
    global oldTheta, btrField
    print("Production Challenge started")
    # initField()
    # print("----")
    # setMPStoField()
    print("====")
    oldTheta = 90
    for pointNumber in range(12 * 0 + 999):
        print(pointNumber)
        route = refbox.refboxNavigationRoutes.route
        if (len(route) == 0):
            print("finished")
        else:
            while True:
                point = getNextPoint(pointNumber)
                if (navToPoint(point) == True):
                    break
            print("arrived #", pointNumber + 1, ": point")
            for i in range(4):
                sendBeacon()
                rospy.sleep(2)

def startOpen():
    print("Run demonstration program")
    challengeFlag = False

    btrRobotino.w_goToWall(0.90)
    btrRobotino.w_goToOutputVelt()
    btrRobotino.w_parallelMPS()
    btrRobotino.w_goToWall(0.40)
    btrRobotino.w_robotinoMove(0, 0.1)
    btrRobotino.w_parallelMPS()
    btrRobotino.w_pick_rs()
    btrRobotino.w_move_scan()
    time.sleep(3)
    #btrRobotino.w_put_rs()

# main
#
if __name__ == '__main__':
  args = sys.argv
  topicName = ""
  gazeboFlag = False
  if (len(args) >= 2):
    challenge = args[1]
    if (len(args) >= 3):
      robotNum = int(args[2])
    else:
      robotNum = 1
    if (challenge == "gazebo" or challenge == "gazebo1"):
      topicName = "/robotino" + str(robotNum)
      gazeboFlag = True

  btrOdometry = Odometry()
  btrBeaconCounter = 0
  btrVelocity = Float32MultiArray()

  btrField = [[0 for y in range(5)] for x in range(5)]

  nodeName = "btr2024_" + str(robotNum)
  rospy.init_node(nodeName)
  rospy.Subscriber(topicName + "/odom", Odometry, robotinoOdometry)
  rate = rospy.Rate(10)

  machineReport = MachineReportEntryBTR()
  prepareMachine = SendPrepareMachine() 

  print(topicName)
  btrRobotino = btr2024.btr2024(topicName)
  refbox = btr_refbox.refbox(teamName = "BabyTigers-R", robotNum = robotNum, gazeboFlag = gazeboFlag)

  if (challenge == "reset"):
      goToPoint(-3.5,  1.5, 90)
      goToPoint(-3.5,  0.5, 90)
      goToPoint(pose.x, pose.y, pose.theta)
      exit()

  pose = Pose2D()
  pose.x = -1.0 * robotNum - 1.5
  pose.y = 0.5
  pose.theta = 90
  if (challenge == "grasping"):
      startX =     [ -0.5, -4.5, -0.5]
      startY =     [  0.5,  1.5,  4.5]
      startTheta = [   90,   90,  180]
      pose.x = startX[robotNum - 1]
      pose.y = startY[robotNum - 1]
      pose.theta = startTheta[robotNum - 1]
  if (challenge == "driving" or challenge == "positioning"):
      pose.x = zoneX["S31"]
      pose.y = zoneY["S31"]
      pose.theta = 90
  if (challenge == "gazebo" or challenge == "rcll" or challenge == "test"):
      pose.x = -(pose.x - 2.0)

  print(pose.x, pose.y, pose.theta)
  btrRobotino.w_resetOdometry(pose)
  # time.sleep(3)
  btrRobotino.w_waitOdometry()

  print(challenge)
  challengeFlag = True
  initField()
  # while True:
  while not rospy.is_shutdown():
    # sendBeacon()
    # print("sendBeacon")
    
    if (challenge == "nbr33" and challengeFlag):
        challengeFlag = False
        # goTo S33
        goToPoint(zoneX["S33"], zoneY["S33"], 90)
        for j in range(2):
            for i in range(9):
                # btrRobotino.w_findMPS()
                w_findMPS()
                btrRobotino.w_robotinoTurnAbs(45 * i)
            print(j)
            time.sleep(3)
        
        goToPoint(zoneX["S31"], zoneY["S31"], 90)
        break

    if (challenge == "exploration" and challengeFlag and refbox.refboxGamePhase == 20 ):
    # if (challenge == "gazebo1" and challengeFlag):
        challengeFlag = False
        # goTo S32
        goToPoint(zoneX["S32"], zoneY["S32"], 90)
        for i in range(5):
            w_findMPS()
            btrRobotino.w_robotinoTurnAbs(45 * i)
        # goTo S34
        navPoint = Pose2D()
        for ZONE in ["S34", "S44", "S42", "S22", "S24", "S32"]:
            navPoint.x = zoneX[ZONE]
            navPoint.y = zoneY[ZONE]
            navPoint.theta = 90
            navToPoint(navPoint)
            for i in range(9):
                w_findMPS()
                btrRobotino.w_robotinoTurnAbs(45 * i)

    if (challenge == "gripping" and challengeFlag):
        slotNo = 3
        if (True):
            moveGo   = [-0.100, -0.220, -0.305]
            moveBack = [ 0.105,  0.220,  0.310]
            btrRobotino.w_goToInputVelt()
            btrRobotino.w_robotinoMove(0,  moveGo[  slotNo - 1])
            btrRobotino.w_getWork()
            # btrRobotino.w_goToInputVelt()
            btrRobotino.w_robotinoMove(0,  moveBack[slotNo - 1])
            btrRobotino.w_putWork()
        challengeFlag = False

    if (challenge == "graspingTest" and challengeFlag):
        startGrasping()
        challengeFlag = False
        break

    if (challenge == "driving" and challengeFlag):
        print("startDriving for JapanOpen2020")
        targetZone =  ["S31", "S35", "S15", "S13", "S33", "S31", "S31", "S31"]
        #                            Target1               Target2
        targetAngle = [   90,     0,   270,   180,   270,    90,    90,    90]
        sleepTime   = [    0,     5,     0,     5,     0,     5,     0,     1]
        for number in range(len(targetZone)):
            print(targetZone[number])
            x = zoneX[targetZone[number]]
            y = zoneY[targetZone[number]]
            theta = targetAngle[number]
            goToPoint(x, y, theta)
            time.sleep(sleepTime[number])

        challengeFlag = False
        break

    if (challenge == "positioning" and challengeFlag):
        print("startPositioning for JapanOpen2020")
        #
        # MPSZone, MPSAngle, firstSide, turn
        #
        MPSZone = "S34" # Input !!!
        MPSAngle = 180  # Input !!!
        firstSide = "input" # Check!!
        turn = "clock" # Check!!

        # goTo S322
        goToPoint(zoneX["S32"], zoneY["S32"], 90)

        if (firstSide == "input"):
            MPSx = zoneX[MPSZone] + inputX[MPSAngle]
            MPSy = zoneY[MPSZone] + inputY[MPSAngle]
            theta = MPSAngle + 180
        else:
            MPSx = zoneX[MPSZone] + outputX[MPSAngle]
            MPSy = zoneY[MPSZone] + outputY[MPSAngle]
            theta = MPSAngle
        
        goToPoint(MPSx, MPSy, theta)
        btrRobotino.w_goToMPSCenter()
        if (firstSide == "input"):
            print("wait")
            time.sleep(10)

        btrRobotino.w_goToWall(20)
        if (turn == "clock"):
            btrRobotino.w_turnClockwise()
        else:
            btrRobotino.w_turnCounterClockwise()
        btrRobotino.w_goToMPSCenter()
        print("wait")
        time.sleep(10)
        
        btrRobotino.w_goToWall(20)
        if (turn == "clock"):
            btrRobotino.w_turnCounterClockwise()
        else:
            btrRobotino.w_turnClockwise()
        if (firstSide == "output"):
            btrRobotino.w_goToMPSCenter() 
            print("wait")
            time.sleep(10)
        
        theta = 270
        goToPoint(MPSx, MPSy, theta)

        # goTo S32 & S31
        goToPoint(zoneX["S32"], zoneY["S32"], 270)
        goToPoint(zoneX["S31"], zoneY["S31"], 90)
        challengeFlag = False
        break

    if (refbox.refboxGamePhase == 30 and challenge == "grasping" and challengeFlag):
        startGrasping()
        challengeFlag = False
        break

    if (challenge == "navigationTest" and challengeFlag):
        startNavigation()
        challengeFlag = False
        break

    if (refbox.refboxGamePhase == 30 and challenge == "navigation" and challengeFlag):
        if (refbox.refboxMachineInfoFlag and refbox.refboxNavigationRoutesFlag):
            startNavigation()
            challengeFlag = False
            break

    # send machine prepare command
    if (refbox.refboxGamePhase == 30 and challenge == "" ):
        # make C0
        # which requires get base with cap from shelf at C-CS1, 
        #                Retrieve cap at C-CS1,
        #                bring base without cap to C-RS1,
        #                get base at C-BS,
        #                bring base to C-CS1,
        #                Mount cap at C-CS1,
        #                bring it to C-DS corresponded by order it.

        if (refbox.refboxTime.sec ==   5):
            prepareMachine.machine = "C-CS1"
            prepareMachine.cs_operation = 1 # CS_OP_RETRIEVE_CAP
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
        if (refbox.refboxTime.sec ==  30):
            prepareMachine.machine = "C-BS"
            prepareMachine.bs_side = 1  # INPUT or OUTPUT side
            prepareMachine.bs_base_color = 1 # BASE COLOR
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
        if (refbox.refboxTime.sec ==  60):
            prepareMachine.machine = "C-CS1"
            prepareMachine.cs_operation = 0 # CS_OP_MOUNT_CAP
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
        if (refbox.refboxTime.sec ==  90):
            prepareMachine.machine = "C-DS"
            prepareMachine.ds_order_id = 1 # ORDER ID
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
            
    if ( challenge == "gazebo"):
        sendBeacon()
        #/ print(challenge)
        if (rebox.refboxGamePhase == 30 and challengeFlag):
            # make c0
            print(refbox.refboxMachineInfo)
           

    if ( challenge == "test" and challengeFlag):
        challengeFlag = False
        refbox.sendBeacon()
        
        goToPoint(zoneX["51"], zoneY["51"],  90)
        goToPoint(zoneX["52"], zoneY["52"],   0)

        # if (btrRobotino.w_findMPS() == True):
        if (w_findMPS() == True):
          btrRobotino.w_goToOutputVelt()
        # btrRobotino.w_goToWall(0.015 + 0.020)
        # btrRobotino.w_parallelMPS()
        # btrRobotino.w_findMPS()

    if (challenge == "test_by_c920"):
        startGrasping_by_c920()
        challengeFlag = False
        break

    if (challenge == "test_C0"):
        graspTransparent()
        challengeFlag = False
        break
    
    if (challenge == "testOpen"):
        startOpen()
        challengeFlag = False
        break
    
    if (challenge == "beacon"):
        refbox.sendBeacon()
        print("Game status is ", refbox.refboxGamePhase)

    if (challenge == "clockwise"):
        btrRobotino.w_turnClockwise()

    if (challenge == "camera"):
        btrRobotino.w_goToInputVelt()
        btrRobotino.w_parallelMPS()
        btrRobotino.w_goToWall(0.4)

    refbox.sendBeacon()
    rate.sleep()


