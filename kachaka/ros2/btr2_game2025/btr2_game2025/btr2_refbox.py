#!/usr/bin/python
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import quaternion
import numpy
import math
# import tf
# from geometry_msgs.msg import Twist, TwistStamped

import refbox_msgs
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, Vector3
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, Int16, UInt32, String, \
                         Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, Empty
from nav_msgs.msg import Odometry
from refbox_msgs.msg import BeaconSignal, ExplorationInfo, \
                            ExplorationSignal, ExplorationZone, GameState, \
                            LightSpec, MachineInfo, Machine, \
                            MachineReportEntry, MachineReportEntryBTR, \
                            MachineReportInfo, OrderInfo, Order, \
                            ProductColor, RingInfo, Ring, Team, Time, \
                            NavigationRoutes, Route
from refbox_msgs.srv import SendBeaconSignal, SendMachineReport, \
                            SendMachineReportBTR, SendPrepareMachine

class refbox(Node):
    def __init__(self, teamName = "Babytigers-R", robotNum = 0, gazeboFlag = False, challenge = "test"):
        self.teamName = teamName
        self.robotNum = robotNum
        self.topicName = ""
        self.challenge = challenge
        if (gazeboFlag):
            self.topicName = "/robot" + str(robotNum)
        super().__init__('btr2_refbox')


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
        self.beaconSignalTime = time.time()

        self.sub01 = self.create_subscription(BeaconSignal, "/rcll/beacon", self.beaconSignal, 10)
        self.sub02 = self.create_subscription(GameState, "/rcll/game_state", self.gameState, 10)
        self.sub03 = self.create_subscription(NavigationRoutes, "/rcll/routes_info", self.navigationRoutes, 100)
        self.sub04 = self.create_subscription(ExplorationInfo, "/rcll/exploration_info", self.explorationInfo, 10)
        self.sub05 = self.create_subscription(MachineInfo, "/rcll/machine_info", self.machineInfo, 10)
        self.sub06 = self.create_subscription(MachineReportInfo, "/rcll/machine_report_info", self.machineReportInfo, 10)
        self.sub07 = self.create_subscription(OrderInfo, "/rcll/order_info", self.orderInfo, 10)
        self.sub08 = self.create_subscription(RingInfo, "/rcll/ring_info", self.ringInfo, 10)
        self.sub09 = self.create_subscription(Odometry, self.topicName + "/odom", self.robotOdometryFunction, 10)

        self.cli01 = self.create_client(SendBeaconSignal, '/rcll/send_beacon')
        self.cli02 = self.create_client(SendMachineReportBTR, '/rcll/send_machine_report')
        self.cli03 = self.create_client(SendPrepareMachine, '/rcll/send_prepare_machine')
        while not self.cli01.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /rcll/send_beacon not available, waiting again...')
        while not self.cli02.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /rcll/send_machine_report not available, waiting again...')
        while not self.cli03.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /rcll/send_prepare_machine not available, waiting again...')
 
        self.req01 = SendBeaconSignal.Request()
        self.req02 = SendMachineReportBTR.Request()
        self.req03 = SendPrepareMachine.Request()

        # self.machineReport = MachineReportEntryBTR()
        # self.prepareMachine = SendPrepareMachine()

        # self.spin_thread = threading.Thread(target=self.sendBeacon, daemon=True)
        # self.spin_thread.start()

    def beaconSignal(self, data):
        self.refboxBeaconSignal = data
        # print("BeaconSignal: ", data)

    def explorationInfo(self, data):
        self.refboxExplorationInfo = data
        print("ExplorationInfo: ", data)

    def gameState(self, data):
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
        if (self.refboxTeamCyan == self.teamName):
            self.teamColor = 1
            self.teamColorName = "C"
        else:
            self.teamColor = 2
            self.teamColorName = "M"
        self.sendBeacon()
        # print("GameState: ", data)

    def machineInfo(self, data):
        self.refboxMachineInfo = data
        self.refboxMachineInfoFlag = True
        print("MachineInfo: ", data)

    def machineReportInfo(self, data):
        self.refboxMachineReportInfo = data
        self.refboxMachineReportInfoFlag = True
        print("MachineReportInfo: ", data)

    def orderInfo(self, data):
        self.refboxOrderInfo = data
        self.refboxOrderInfoFlag = True
        print("OrderInfo: ", data)

    def ringInfo(self, data):
        self.refboxRingInfo = data
        self.refboxRingInfoFlag = True
        print("RingInfo: ", data)

    def navigationRoutes(self, data):
        self.refboxNavigationRoutes = data
        self.refboxNavigationRoutesFlag = True
        print("NavigaionRoutes: ", data)

    #
    # get robot odometry data
    #
    def quaternion_to_euler(self, quaternion):
        # e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        # return Vector3(x=e[0], y=e[1], z=e[2])
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


    def robotOdometryFunction(self, data):
        quat = self.quaternion_to_euler(data.pose.pose.orientation)
        self.robotOdometry = data
        self.robotOdometryFlag = True
        # print("odometry")
        # self.robotOdometry.pose.pose.position.z = quat[2] # / math.pi * 180
        # self.robotOdometry.pose.pose.position.z = self.robotOdometry.pose.pose.position.z # / math.pi * 180
        # trOdometry = data
        
        # self.robotBeaconCounter +=1
        # if (self.robotBeaconCounter > 5):
        #     self.sendBeacon()
        #     self.robotBeaconCounter = 0
        self.sendBeacon()
        # print("[odometry]: z", self.robotOdometry.pose.pose.position.z)

    #
    # send information to RefBox
    #
    def sendBeacon(self):
        if (self.robotOdometryFlag == False):
            print("not received odometry information")
            self.robotOdometry.pose.pose.position.x = 0.0
            self.robotOdometry.pose.pose.position.y = 0.0
            self.robotOdometry.pose.pose.orientation.x = 0.0
            self.robotOdometry.pose.pose.orientation.y = 0.0
            self.robotOdometry.pose.pose.orientation.z = -0.5999129245831818
            self.robotOdometry.pose.pose.orientation.w = 0.8000652991587959
            # return
        # if robotNum == 0, don't send the Beacon.
        if (self.robotNum == 0):
            return

        # self.robotBeaconCounter +=1
        # if (self.robotBeaconCounter > 5):
        #     self.robotBeaconCounter = 0
        # else:
        #     return
        self.lastBeaconSignalTime = self.beaconSignalTime
        self.nowBeaconSignalTime = time.time()
        if (int(self.lastBeaconSignalTime) == int(self.nowBeaconSignalTime)):
            return
        self.beaconSignalTime = self.nowBeaconSignalTime
        print(f"[sendBeacon] {self.nowBeaconSignalTime}")
        print(f"[sendBeacon] {self.robotOdometry.pose.pose.position}")

        beacon = self.req01
        beacon.header = Header()

        # for poseStamped()
        beacon.pose = PoseStamped()
        beacon.pose.pose.position.x = self.robotOdometry.pose.pose.position.x # / 1000
        beacon.pose.pose.position.y = self.robotOdometry.pose.pose.position.y # / 1000
        beacon.pose.pose.position.z = 0.0 + self.robotOdometry.pose.pose.position.z # Chest
        beacon.pose.pose.orientation.x = self.robotOdometry.pose.pose.orientation.x
        beacon.pose.pose.orientation.y = self.robotOdometry.pose.pose.orientation.y
        beacon.pose.pose.orientation.z = self.robotOdometry.pose.pose.orientation.z
        beacon.pose.pose.orientation.w = self.robotOdometry.pose.pose.orientation.w
        # beacon.header.seq = 1
        # beacon.header.stamp = rospy.Time.now()
        timeNow = self.get_clock().now()
        # print(time)
        # print(beacon.header.stamp)
        beacon.header.stamp.sec = timeNow.nanoseconds // 1000000000
        beacon.header.stamp.nanosec = timeNow.nanoseconds % 100000000
        beacon.header.frame_id = self.teamName
        # beacon.pose.header.seq = 1
        beacon.pose.header.stamp.sec = timeNow.nanoseconds // 1000000000 # rospy.Time.now()
        beacon.pose.header.stamp.nanosec = timeNow.nanoseconds % 1000000000
        beacon.pose.header.frame_id = "robot1"

        self.req01 = beacon
        self.future = self.cli01.call_async(self.req01)
        # print("spin_until_future_complete")
        # rclpy.spin_until_future_complete(self, self.future, timeout_sec = 0.1)
        # rclpy.spin_once(self, timeout_sec = 0.1)
        # print("spin_once")
        # rclpy.spin_once(self, timeouti_sec = 0.01)
        # print("sendBeacon: ", self.future.result())
        return self.future.result()

    def sendMachineReport(self, report):
        # sendReport = SendMachineReport() 
        sendReport = self.req02
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

        self.req02 = sendRepomrt
        # rospy.wait_for_service('/rcll/send_machine_report')
        self.future = self.cli02.call_async(self.req02)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec = 0.1)
        # rclpy.spin_once(self, timeout_sec = 0.1)
        return self.future.result()
        #try:
        #    self.refboxMachineReport = rospy.ServiceProxy('/rcll/send_machine_report', SendMachineReportBTR)
        #    resp1 = self.refboxMachineReport(sendReport.team_color, sendReport.machines)
        #    # print("resp: ", resp1)
        #    return resp1
        #except rospy.ServiceException as e:
        #    print("Service call failed: %s"%e)

    def sendPrepareMachine(self,data):
        prepare = self.req03
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
        # rospy.wait_for_service('/rcll/send_prepare_machine')
        # try:
        #     self.refboxPrepareMachine = rospy.ServiceProxy('/rcll/send_prepare_machine', SendPrepareMachine)
        #     resp1 = self.refboxPrepareMachine(prepare.machine, prepare.wait, prepare.bs_side, prepare.bs_base_color, prepare.ds_order_id, prepare.rs_ring_color, prepare.cs_operation)
        #     return resp1
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s"%e)
    
        self.req03 = prepare
        self.future = self.cli03.call_async(self.req03)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec = 0.1)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    btr2_refbox = refbox(teamName = "Babytigers-R", robotNum = 1, gazeboFlag = False)
    # while True:
    rclpy.spin(btr2_refbox)
    # btr2_refbox.sendBeacon()

    while rclpy.ok():
        rclpy.spin_once(btr2_refbox) # コールバック関数を1回だけ呼び出す
        if btr2_refbox.future.done():  #  Futureがキャンセルされるか、結果を得るたらfuture.done（）がTrueになる。
            try:
                response = btr2_refbox.future.result() # サーバーから非同期的に送られてきたレスポンス
            except Exception as e:                                         #  エラー時の処理
                btr2_refbox.get_logger().info(
                    'Service call failed %r' % (e,))
            else:  #  エラーでないときは、端末にレスポンスである亀の名前を表示する
                btr2_refbox.get_logger().info('Response:=%s' % response)
            break

    btr2_refbox.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
