#!/usr/bin/python
import struct
import time
import math
import sys
import rospy
import numpy

import quaternion
import tf
import rcll_ros_msgs
import rcll_btr_msgs
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, PointStamped, Point, \
                              Quaternion, Vector3
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

def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def beaconSignal(data):
    global refboxBeaconSignal
    refboxBeaconSignal = data
    # print("BeaconSignal: ", data)

def explorationInfo(data):
    global refboxExplorationInfo
    refboxExplorationInfo = data
    # print("ExplorationInfo: ", data)

def gameState(data):
    global refboxTime, refboxGameState, refboxGamePhase, \
           refboxPointsMagenta, refboxTeamMagenta, \
           refboxPointCyan, refboxTeamCyan
    refboxTime = data.game_time
    refboxGameState = data.state
    refboxGamePhase = data.phase
    refboxPointsMagenta = data.points_magenta
    refboxTeamMagenta = data.team_magenta
    refboxPointsCyan = data.points_cyan
    refboxTeamCyan = data.team_cyan
    # print("GameState: ", data)
    sendBeacon()

def machineInfo(data):
    global refboxMachineInfo, refboxMachineInfoFlag
    refboxMachineInfo = data
    refboxMachineInfoFlag = True
    # print("MachineInfo: ", data)

def machineReportInfo(data):
    global refboxMachineReportInfo
    refboxMachineReportInfo = data
    # print("MachineReportInfo: ", data)

def orderInfo(data):
    global refboxOrderInfo
    refboxOrderInfo = data
    # print("OrderInfo: ", data)

def ringInfo(data):
    global refboxRingInfo
    refboxRingInfo = data
    # print("RingInfo: ", data)

def navigationRoutes(data):
   global refboxNavigationRoutes, refboxNavigationRoutesFlag
   refboxNavigationRoutes = data
   refboxNavigationRoutesFlag = True
   # print("NavigaionRoutes: ", data)

#
# send information to RefBox
#
def sendBeacon():
    global btrOdometry
    beacon = SendBeaconSignal()
    beacon.header = Header()

    # for poseStamped()
    beacon.pose = PoseStamped()
    beacon.pose.pose.position.x = btrOdometry.pose.pose.position.x # / 1000
    beacon.pose.pose.position.y = btrOdometry.pose.pose.position.y # / 1000
    beacon.pose.pose.position.z = 0
    beacon.pose.pose.orientation.x = btrOdometry.pose.pose.orientation.x
    beacon.pose.pose.orientation.y = btrOdometry.pose.pose.orientation.y
    beacon.pose.pose.orientation.z = btrOdometry.pose.pose.orientation.z
    beacon.pose.pose.orientation.w = btrOdometry.pose.pose.orientation.w
    beacon.header.seq = 1
    beacon.header.stamp = rospy.Time.now()
    beacon.header.frame_id = TEAMNAME
    beacon.pose.header.seq = 1
    beacon.pose.header.stamp = rospy.Time.now()
    beacon.pose.header.frame_id = "robot1"

    rospy.wait_for_service('/rcll/send_beacon')
    try:
        refboxSendBeacon = rospy.ServiceProxy('/rcll/send_beacon', SendBeaconSignal)
        resp1 = refboxSendBeacon(beacon.header, beacon.pose)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def sendMachineReport(report):
    global machineReport
    sendReport = SendMachineReport()
    machineReport = MachineReportEntryBTR
    machineReport.name = report.name
    machineReport.type = report.type
    machineReport.zone = report.zone
    machineReport.rotation = report.rotation
    if (refboxTeamCyan == TEAMNAME):
        sendReport.team_color = 1
    else:
        sendReport.team_color = 2
    MachineReportEntryBTR = [machineReport]
    sendReport.machines = MachineReportEntryBTR
    print("machineReport: ", machineReport)

    rospy.wait_for_service('/rcll/send_machine_report')
    try:
        refboxMachineReport = rospy.ServiceProxy('/rcll/send_machine_report', SendMachineReportBTR)
        resp1 = refboxMachineReport(sendReport.team_color, sendReport.machines)
        # print("resp: ", resp1)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def sendPrepareMachine(data):
    prepare = SendPrepareMachine()
    # prepare.machine = data.machine
    prepare.machine = data.machine
    prepare.bs_side = 0
    prepare.bs_base_color = 0
    prepare.ds_order_id = 0
    prepare.cs_operation = 0
    prepare.rs_ring_color =0
    
    machineType = prepare.machine[2:4]
    print(machineType)
    if (machineType == "BS"):
        prepare.bs_side = data.bs_side
        prepare.bs_base_color =data.bs_base_color
    if (machineType == "DS"):
        prepare.ds_order_id = data.ds_order_id
    if (machineType == "CS"):
        prepare.cs_operation = data.cs_operation
    if (machineType == "RS"):
        prepare.rs_ring_color = data.rs_ring_color
    prepare.wait = data.wait
    rospy.wait_for_service('/rcll/send_prepare_machine')
    try:
        refboxPrepareMachine = rospy.ServiceProxy('/rcll/send_prepare_machine', SendPrepareMachine)
        resp1 = refboxPrepareMachine(prepare.machine, prepare.wait, prepare.bs_side, prepare.bs_base_color, prepare.ds_order_id, prepare.rs_ring_color, prepare.cs_operation)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def robotinoOdometry(data):
    global btrOdometry, btrBeaconCounter
    quat = quaternion_to_euler(Quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))
    btrOdometry = data
    ##btrOdometry.pose.pose.position.z = quat.z / math.pi * 180
    btrOdometry.pose.pose.position.z = btrOdometry.pose.pose.position.z # / math.pi * 180
    # btrOdometry = data
    btrBeaconCounter +=1
    if (btrBeaconCounter > 5):
      sendBeacon()
      btrBeaconCounter = 0

# main
#
if __name__ == '__main__':
    args = sys.argv
    topicName = ""
    challenge=""
    TEAMNAME=""
    robotNum = 1
    # args: TEAMNAME, challengename, RobotNumber
    if (len(args) == 1):
        print("Usage: " + args[0] + " TeamName ChallengeName RobotNumber")
    if (len(args) >= 2):
        TEAMNAME = args[1]
    if (len(args)>=3):
        challenge = args[2]
    if (len(args) >= 4):
        robotNum = int(args[3])
    if (challenge == "gazebo" or challenge == "gazebo1"):
        topicName = "/robotino" + str(robotNum)

    # valiables for refbox
    refboxBeaconSignal = BeaconSignal()
    refboxExplorationInfo = ExplorationInfo()
    refboxExplorationSignal = ExplorationSignal()
    refboxExplorationZone = ExplorationZone()
    refboxGameState = Int8()
    refboxGamePhase = Int8()
    refboxPointsMagenta = UInt32()
    refboxTeamMagenta = String()
    refboxPointsCyan = UInt32()
    refboxTeamCyan = String()
    refboxLightSpec = LightSpec()
    refboxMachineInfo = MachineInfo()
    refboxMachine = Machine()
    refboxMachineReportEntry = MachineReportEntryBTR()
    refboxMachineReportInfo = MachineReportInfo()
    refboxOrderInfo = OrderInfo()
    refboxOrder = Order()
    refboxProductColor = ProductColor()
    refboxRingInfo = RingInfo()
    refboxRing = Ring()
    refboxTeam = Team()
    refboxTime = Time()
    refboxNavigationRoutes = NavigationRoutes()
    refboxMachineInfoFlag = False
    refboxNavigationRoutesFlag = False

    btrOdometry = Odometry()
    btrBeaconCounter = 0
    btrVelocity = Float32MultiArray()

    nodeName = "rosRefbox_" + str(robotNum)
    rospy.init_node(nodeName)
    rospy.Subscriber("rcll/beacon", BeaconSignal, beaconSignal)
    rospy.Subscriber("rcll/exploration_info", ExplorationInfo, explorationInfo)
    rospy.Subscriber("rcll/game_state", GameState, gameState)
    rospy.Subscriber("rcll/machine_info", MachineInfo, machineInfo)
    rospy.Subscriber("rcll/machine_report_info", MachineReportInfo, machineReportInfo)
    rospy.Subscriber("rcll/order_info", OrderInfo, orderInfo)
    rospy.Subscriber("rcll/ring_info", RingInfo, ringInfo)
    rospy.Subscriber(topicName + "/odom", Odometry, robotinoOdometry)
    rospy.Subscriber("rcll/routes_info", NavigationRoutes, navigationRoutes)
    rate = rospy.Rate(10)

    machineReport = MachineReportEntryBTR()
    prepareMachine = SendPrepareMachine() 

    print(topicName)

    # while True:
    while not rospy.is_shutdown():
        # sendBeacon()
        # print("sendBeacon")
        sendBeacon()
        rate.sleep()


