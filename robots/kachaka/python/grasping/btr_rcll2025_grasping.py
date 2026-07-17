import os
import sys
sys.path.append("../../../")

import kachaka_api
import math
from kachaka.python.adjust_pose.adjust_X_Y_Zr import adjust_X_Y_Zr
from module_object_detector import module_object_detector
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


grasping_position = [0.0, 0.0, 0.2175] # meter
grasping_tolerance = [0.05, 0.10, 0.02] # meter

initial_angle = None

class grasping():
    def __init__(self):
        rclpy.init()
        self.node = Node("grasping_program")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.topicName = ""
        self.sub01 = self.node.create_subscription(Point, self.topicName + "/btr/centerPoint", self.center_point, 10)
        #self.sub02 = self.node.create_subscription(Point, self.topicName + "/btr/closePoint", 10)
        #self.sub03 = self.node.create_subscription(Point, self.topicName + "/btr/leftPoint", 10)
        #self.sub04 = self.node.create_subscription(Point, self.topicName + "/btr/rightPoint", 10)
        #self.sub05 = self.node.create_subscription(Point, self.topicName + "/btr/forwardPoint", 10)


    def startGrasping(self):
        od = module_object_detector()

        # make kachaka api client
        kachakaIP = os.getenv('kachaka_IP')
        client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")

        # a class for adjusting kachaka pose
        # x_y_zr_adjuster = adjust_X_Y_Zr(client)

        # set for the angle of the kachaka
        pose = client.get_robot_pose()
        initial_angle = pose.theta

        print("#==================#")
        print("start grasping")

        for n in range(3):
            print("------")
            print("repeation: ", n)

            # move to the output side of the machine. 
            # client.move_to_pose(0.0, -2.0, math.pi/2)
            # client.move_to_pose(0.013, -1.878, 0.480)

            # detect belt position and adjust the robot position
            # belt_position = adjust_position(od, x_y_zr_adjuster)

            ###
            client.move_forward(0.05)
            client.move_forward(-0.05)
            rclpy.spin_once(self.node)
            self.adjust_rotation(client)

            od.take_photo()

            belt_position = []
            for j in od.belt_detect():
                belt_position.append(j)
            print(belt_position)
            if belt_position[0] == None:
                position = [float(grasping_position[0]), float(grasping_position[2])]
            else:
                position = [float(belt_position[0]), float(belt_position[2])]

            # grasping
            cmd_myPalletizer("moveG", position)
            
            # move to the input side of the machine. 
            # client.move_to_pose(2.0, -2.0, -math.pi/2)
            # magenta_out2in(client, initial_angle)
            cyan_out2in(client, initial_angle)

            # detect belt position and adjust the robot position
            # belt_position = adjust_position(od, x_y_zr_adjuster)

            ###
            client.move_forward(0.05)
            client.move_forward(-0.05)
            rclpy.spin_once(self.node)
            self.adjust_rotation(client)

            od.take_photo()

            belt_position = []
            for j in od.belt_detect():
                belt_position.append(j)
            print(belt_position)
            if belt_position[0] == None:
                position = [float(grasping_position[0]), float(grasping_position[2])]
            else:
                position = [float(belt_position[0]), float(belt_position[2])]

            # releasing
            cmd_myPalletizer("moveR", position)
        
            # return to the output side
            # magenta_in2out(client, initial_angle)
            cyan_in2out(client, initial_angle)

        print("#==================#")
        
    def center_point(self, data):
        self.p_center = math.radians(data.z)
        # print(self.p_center)

    def adjust_rotation(self, client):
        print("mps rotation: ", self.p_center)
        if self.p_center != math.nan:
            client.rotate_in_place(self.p_center) 
       

def magenta_out2in(client, initial_angle):
    speed = 0.3
    client.move_forward(0.75, speed=speed)
    kachaka_rotate_in_place(client, initial_angle - math.pi / 2.0)
    client.move_forward(0.90, speed=speed)
    kachaka_rotate_in_place(client, initial_angle - math.pi / 2.0)
    client.move_forward(0.75, speed=speed)

def magenta_in2out(client, initial_angle):
    speed = 0.3
    client.move_forward(-0.75, speed=speed)
    kachaka_rotate_in_place(client, initial_angle - math.pi / 2.0)
    client.move_forward(-0.90, speed=speed)
    kachaka_rotate_in_place(client, initial_angle)
    client.move_forward(-0.75, speed=speed)

def cyan_out2in(client, initial_angle):
    speed = 0.3
    client.move_forward(-0.75, speed=speed)
    kachaka_rotate_in_place(client, initial_angle + math.pi / 2.0)
    client.move_forward(-0.90, speed=speed)
    kachaka_rotate_in_place(client, initial_angle + math.pi)
    client.move_forward(-0.75, speed=speed)

def cyan_in2out(client, initial_angle):
    speed = 0.3
    client.move_forward(0.75, speed=speed)
    kachaka_rotate_in_place(client, initial_angle + math.pi / 2.0)
    client.move_forward(0.90, speed=speed)
    kachaka_rotate_in_place(client, initial_angle)
    client.move_forward(0.75, speed=speed)

def kachaka_rotate_in_place(client, angle):
    for i in range(3):
        pose = client.get_robot_pose()
        now_angle = pose.theta
        # if (angle - now_angle) > (math.pi/180):
        target_angle = normalize_angle(angle - now_angle)
        client.rotate_in_place(target_angle)

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def calc_point(point, x, y):
    theta = point.theta

"""
def adjust_position(od, adjuster):
    x_y_zr_adjuster = adjuster
    # detect belt position and adjust the robot position
    for i in range(10):
        od.take_photo()
        belt_position = []
        for j in od.belt_detect():
            belt_position.append(i)

        if belt_position[0] == None:
            x_y_zr_adjuster.adjust_Y(-(grasping_tolerance[1]))

        else:
            # calculate the position errors
            position_error = []
            for j in range(len(grasping_position)):
                position_error.append(belt_position[j] - grasping_position[j])

            if (abs(position_error[0]) <= grasping_tolerance[0]) and (abs(position_error[1]) <= grasping_tolerance[1]) and (abs(position_error[2]) <= grasping_tolerance[2]):
                return belt_position

            else:
                if position_error[2] != 0:
                    x_y_zr_adjuster.adjust_Y((position_error[2] / abs(position_error[2])) / 20) # move 0.05 m
                if position_error[0] != 0:
                    x_y_zr_adjuster.adjust_X((position_error[0] / abs(position_error[0])) / 20) # move 0.05 m

    print("Could not adjust the position")
    return grasping_position
"""

def cmd_myPalletizer(mode, position):
    CMD = "ssh palletizer-0 -l er -i /home/ryukoku/.ssh/id_rsa python3 /home/er/git/rcll/palletizer/MyPallBTR.py"
    if mode == "moveG":
        cmd = CMD + " RCLL2025_moveG {} {}".format(*position)
    elif mode == "moveR":
        cmd = CMD + " RCLL2025_moveR {} {}".format(*position)
    print(cmd)
    os.system(cmd)



def main():
    inst = grasping()
    inst.startGrasping()

if __name__ == "__main__":
    main()
