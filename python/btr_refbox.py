#!/usr/bin/python
import time
import sys
import rospy
import quaternion
import tf

import rcll_ros_msgs
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, Vector3
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, Int16, UInt32, String, \
                         Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse, Empty, EmptyResponse
from nav_msgs.msg import Odometry
from rcll_ros_msgs.msg import BeaconSignal, ExplorationInfo, \
                              ExplorationSignal, ExplorationZone, GameState, \
                              LightSpec, MachineInfo, Machine, \
                              MachineReportEntry, MachineReportEntryBTR, \
                              MachineReportInfo, OrderInfo, Order, \
                              ProductColor, RingInfo, Ring, Team, Time, \
                              NavigationRoutes, Route
from rcll_ros_msgs.srv import SendBeaconSignal, SendMachineReport, \
                              SendMachineReportBTR, SendPrepareMachine

class refbox(object):
    def __init__(self, teamName = "BabyTigers-R", robotNum = 0, gazeboFlag = False):
        self.teamName = teamName
        self.robotNum = robotNum
        self.topicName = ""
        if (gazeboFlag):
            self.topicName = "/robotino" + str(robotNum)

        # valiables for refbox
        self.refboxBeaconSignal = BeaconSignal()
        self.refboxExplorationInfo = ExplorationInfo()
        self.refboxExplorationSignal = ExplorationSignal()
        self.refboxExplorationZone = ExplorationZone()
        self.refboxGameTime = Time()
        self.refboxGameState = Int8()
        self.refboxGamePhase = Int8()
        self.refboxPointsMagenta = UInt32()
        self.refboxTeamMagenta = String()
        self.refboxPointsCyan = UInt32()
        self.refboxTeamCyan = String()
        self.refboxFieldHeight = UInt32()
        self.refboxFieldWidth = UInt32()
        self.refboxFieldMirrored = Bool()
        self.refboxLightSpec = LightSpec()
        self.refboxMachineInfo = MachineInfo()
        self.refboxMachine = Machine()
        self.refboxMachineReportEntry = MachineReportEntryBTR()
        self.refboxMachineReportInfo = MachineReportInfo()
        self.refboxOrderInfo = OrderInfo()
        self.refboxOrder = Order()
        self.refboxProductColor = ProductColor()
        self.refboxRingInfo = RingInfo()
        self.refboxRing = Ring()
        self.refboxTeam = Team()
        self.refboxNavigationRoutes = NavigationRoutes()
        self.refboxGameStateFlag = False
        self.refboxMachineInfoFlag = False
        self.refboxMachineReportInfoFlag = False
        self.refboxOrderInfoFlag = False
        self.refboxRingInfoFlag = False
        self.refboxNavigationRoutesFlag = False
        self.teamColor = 0
        self.teamColorName = ""

        self.robotOdometry = Odometry()
        self.robotOdometryFlag = False
        self.robotBeaconCounter = 0
        self.robotVelocity = Float32MultiArray()

        # self.nodeName = "rosRefbox_" + str(robotNum)
        # rospy.init_node(nodeName)

        self.sub01 = rospy.Subscriber("/rcll/beacon", BeaconSignal, self.beaconSignal)
        self.sub02 = rospy.Subscriber("/rcll/exploration_info", ExplorationInfo, self.explorationInfo)
        self.sub03 = rospy.Subscriber("/rcll/game_state", GameState, self.gameState)
        self.sub04 = rospy.Subscriber("/rcll/machine_info", MachineInfo, self.machineInfo)
        self.sub05 = rospy.Subscriber("/rcll/machine_report_info", MachineReportInfo, self.machineReportInfo)
        self.sub06 = rospy.Subscriber("/rcll/order_info", OrderInfo, self.orderInfo)
        self.sub07 = rospy.Subscriber("/rcll/ring_info", RingInfo, self.ringInfo)
        self.sub08 = rospy.Subscriber(self.topicName + "/odom", Odometry, self.robotinoOdometry)
        self.sub09 = rospy.Subscriber("/rcll/routes_info", NavigationRoutes, self.navigationRoutes)
        self.rate =rospy.Rate(10)

        self.machineReport = MachineReportEntryBTR()
        self.prepareMachine = SendPrepareMachine()


    def beaconSignal(self, data):
        # global refboxBeaconSignal
        self.refboxBeaconSignal = data
        # print("BeaconSignal: ", data)

    def explorationInfo(self, data):
        # global refboxExplorationInfo
        self.refboxExplorationInfo = data
        # print("ExplorationInfo: ", data)

    def gameState(self, data):
        # global refboxGameTime, refboxGameState, refboxGamePhase, \
        #    refboxPointsMagenta, refboxTeamMagenta, \
        #    refboxPointsCyan, refboxTeamCyan
        self.refboxGameTime = data.game_time
        self.refboxGameState = data.state
        self.refboxGamePhase = data.phase
        self.refboxPointsMagenta = data.points_magenta
        self.refboxTeamMagenta = data.team_magenta
        self.refboxPointsCyan = data.points_cyan
        self.refboxTeamCyan = data.team_cyan
        self.refboxFieldHeight = data.field_height
        self.refboxFieldWidth = data.field_width
        self.refboxFieldMirrored = data.field_mirrored
        self.refboxGameStateFlag = True
        # print("GameState: ", data)
        if (self.refboxTeamCyan == self.teamName):
            self.teamColor = 1
            self.teamColorName = "C"
        else:
            self.teamColor = 2
            self.teamColorName = "M"

        self.sendBeacon()

    def machineInfo(self, data):
        # global refboxMachineInfo, refboxMachineInfoFlag
        self.refboxMachineInfo = data
        self.refboxMachineInfoFlag = True
        # print("MachineInfo: ", data)

    def machineReportInfo(self, data):
        # global refboxMachineReportInfo
        self.refboxMachineReportInfo = data
        self.refboxMachineReportInfoFlag = True
        # print("MachineReportInfo: ", data)

    def orderInfo(self, data):
        # global refboxOrderInfo
        self.refboxOrderInfo = data
        self.refboxOrderInfoFlag = True
        # print("OrderInfo: ", data)

    def ringInfo(self, data):
        # global refboxRingInfo
        self.refboxRingInfo = data
        self.refboxRingInfoFlag = True
        # print("RingInfo: ", data)

    def navigationRoutes(self, data):
        # global refboxNavigationRoutes
        self.refboxNavigationRoutes = data
        self.refboxNavigationRoutesFlag = True
        # print("NavigaionRoutes: ", data)

    #
    # get robot odometry data
    # 
    def quaternion_to_euler(self, quaternion):
        """Convert Quaternion to Euler Angles

        quarternion: geometry_msgs/Quaternion
        euler: geometry_msgs/Vector3
        """
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return Vector3(x=e[0], y=e[1], z=e[2])

    def robotinoOdometry(self, data):
        # global robotOdometry, robotBeaconCounter
        # quat = self.quaternion_to_euler(Quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))
        self.robotOdometry = data
        self.robotOdometryFlag = True
        # print("odometry")
        ## self.robotOdometry.pose.pose.position.z = quat.z / math.pi * 180
        self.robotOdometry.pose.pose.position.z = self.robotOdometry.pose.pose.position.z # / math.pi * 180
        # trOdometry = data
        self.robotBeaconCounter +=1
        if (self.robotBeaconCounter > 5):
            self.sendBeacon()
            self.robotBeaconCounter = 0

    #
    # send information to RefBox
    #
    def sendBeacon(self):
        # global robotOdometry
        if (self.robotOdometryFlag == False):
            print("not received odometry information")
            return
        # if robotNum == 0, don't send the Beacon.
        if (self.robotNum == 0):
            return
        beacon = SendBeaconSignal()
        beacon.header = Header()

        # for poseStamped()
        beacon.pose = PoseStamped()
        beacon.pose.pose.position.x = self.robotOdometry.pose.pose.position.x # / 1000
        beacon.pose.pose.position.y = self.robotOdometry.pose.pose.position.y # / 1000
        beacon.pose.pose.position.z = 0
        beacon.pose.pose.orientation.x = self.robotOdometry.pose.pose.orientation.x
        beacon.pose.pose.orientation.y = self.robotOdometry.pose.pose.orientation.y
        beacon.pose.pose.orientation.z = self.robotOdometry.pose.pose.orientation.z
        beacon.pose.pose.orientation.w = self.robotOdometry.pose.pose.orientation.w
        beacon.header.seq = 1
        beacon.header.stamp = rospy.Time.now()
        beacon.header.frame_id = self.teamName
        beacon.pose.header.seq = 1
        beacon.pose.header.stamp = rospy.Time.now()
        beacon.pose.header.frame_id = "robot1"

        rospy.wait_for_service('/rcll/send_beacon')
        try:
            self.refboxSendBeacon = rospy.ServiceProxy('/rcll/send_beacon', SendBeaconSignal)
            resp1 = self.refboxSendBeacon(beacon.header, beacon.pose)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def sendMachineReport(self, report):
        # global machineReport
        sendReport = SendMachineReport()
        machineReport = MachineReportEntryBTR()
        machineReport.name = report.name
        machineReport.type = report.type
        machineReport.zone = report.zone
        machineReport.rotation = report.rotation
        if (self.refboxTeamCyan == self.teamName):
            sendReport.team_color = 1
        else:
            sendReport.team_color = 2
        # MachineReportEntryBTR = [machineReport]
        # sendReport.machines = MachineReportEntryBTR
        sendReport.machines = [machineReport]
        print("machineReport: ", machineReport)

        rospy.wait_for_service('/rcll/send_machine_report')
        try:
            self.refboxMachineReport = rospy.ServiceProxy('/rcll/send_machine_report', SendMachineReportBTR)
            resp1 = self.refboxMachineReport(sendReport.team_color, sendReport.machines)
            # print("resp: ", resp1)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def sendPrepareMachine(self,data):
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
            self.refboxPrepareMachine = rospy.ServiceProxy('/rcll/send_prepare_machine', SendPrepareMachine)
            resp1 = self.refboxPrepareMachine(prepare.machine, prepare.wait, prepare.bs_side, prepare.bs_base_color, prepare.ds_order_id, prepare.rs_ring_color, prepare.cs_operation)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

# main
#
if __name__ == '__main__':

    rospy.init_node('refboxSample')
    refbox = refbox(teamName = "BabyTigers-R", robotNum = 1, gazeboFlag = False)
    # while True:
    while not rospy.is_shutdown():
        # sendBeacon()
        # print("sendBeacon")
        refbox.sendBeacon()
        refbox.rate.sleep()


