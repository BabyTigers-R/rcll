#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket 
import time
import threading
import os
import sys
import textwrap
import serial
import serial.tools.list_ports

from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD

###

def create_list(n):
    # read external file
    external_file = open(sys.argv[n],'r')
    external_string = external_file.read()
    # create list, list style is like "[[[a,b,c,d,e,f],g,h,i],...]"
    external_string = external_string.strip().splitlines()

    states_list = list()
    for i in external_string:
        if i != '' and i.strip()[0] != "#":
            states_list.append([[float(j) for j in i.strip().split(",")[0:4]],int(i.strip().split(",")[4]),float(i.strip().split(",")[5]),int(i.strip().split(",")[6])])

    external_file.close()
    return states_list
6
def modify_joint(l:list):
    # read arg
    external_arg = float(sys.argv[3])
    modified_angle = external_arg * 3.5 
    if external_arg <= 3 and external_arg >= -3:
        for i in range(len(l)):
            if i == 0:
                pass
            elif i == len(l)-1:
                pass
            else:
                l[i][0][0] = l[i][0][0] + modified_angle
                l[i][0][5] = l[i][0][5] + modified_angle
                if external_arg < 0:
                    l[i][0][2] = l[i][0][2] + (abs(modified_angle) / 3.3)
                    l[i][0][3] = l[i][0][3] + (abs(modified_angle) / 3.3)
                else:
                    l[i][0][2] = l[i][0][2] - (modified_angle / 3.5)
                    l[i][0][3] = l[i][0][3] - (modified_angle / 3.5)

    return l


###

serialPort = "/dev/ttyAMA0"
# serialBaud = "115200"
serialBaud = "1000000"

class MycobotBTR(object):
    def __init__(self):
        self.mycobot = None

    def run(self, command):
        self.connect_mycobot()
        self.mycobot.power_on()
        self.send_color("green")
        if command == "init":
            self.gripperClose()
            self.mycobot.send_angles([76.2, 47.28, 36.56, 71.98], 30)
            time.sleep(1)
        if command == "status":
            self.getStatus()
        if command == "teach":
            self.teaching()
        if command == "open":
            self.gripperOpen()
        if command == "close":
            self.gripperClose()
        if command == "gripper":
            print(self.mycobot.get_gripper_value(), self.mycobot.is_gripper_moving())
        if command == "move_Work":
            self.Arm_states_list = create_list(2)
            #self.Arm_states_list = modify_joint(self.Arm_states_list)
            self.move_Work()
        self.disconnect_mycobot()

    def move_Work(self):
        #self.gripperGoInit()
        #self.gripperGoStart()
        #self.gripperOpen()
        #self.gripperGoAbove()
        #self.gripperGoWork()
        #self.gripperClose()
        #time.sleep(1)
        #self.gripperGoAbove()
        #self.gripperGoStart()
        #self.gripperGoInit()
        for i in range(len(self.Arm_states_list)):
            self.mycobot.send_angles(self.Arm_states_list[i][0],self.Arm_states_list[i][1])
            time.sleep(self.Arm_states_list[i][2])
            if self.Arm_states_list[i][3] == 0:
                self.gripperOpen()
                #self.mycobot.set_gripper_ini()
            elif self.Arm_states_list[i][3] == 1:
                self.gripperClose()

    def gripperOpen(self):
        # self.mycobot.set_gripper_value(2047, 10)
        self.mycobot.set_gripper_state(0, 100)
        time.sleep(0.5)

    def gripperClose(self):
        # self.mycobot.set_gripper_value(1400, 10)
        self.mycobot.set_gripper_state(1, 100)
        time.sleep(0.5)

    def teaching(self):
        self.send_color("blue")
        #self.mycobot.release_all_servos()
        self.motor_off()
        self.count_10s()
        #for i in range(1, 4):
        #    self.mycobot.focus_servo(i)
        self.motor_on()
        time.sleep(1)
        self.getStatus()

    def getStatus(self):
        print("Angles: ", self.mycobot.get_angles())
        print("Radians: ", self.mycobot.get_radians())

    def motor_on(self):
        self.mycobot.power_on()

    def motor_off(self):
        self.mycobot.power_off()

    # ============================================================
    # Connect method
    # ============================================================
    def connect_mycobot(self):
        global serialPort, serialBaud
        self.port = port = serialPort
        self.baud = baud = serialBaud
        baud = int(baud)

        # self.mycobot = MyCobot(PI_PORT, PI_BAUD)
        self.mycobot = MyCobot(port, baud)
        # self.mycobot = MyCobot("/dev/cu.usbserial-0213245D", 115200)

    def disconnect_mycobot(self):
        if not self.has_mycobot():
            return

        del self.mycobot
        self.mycobot = None

    # ============================================================
    #  Function method
    # ============================================================
    def count_10s(self):
        for i in range(8):
            time.sleep(0.5)
            self.send_color("green")
            time.sleep(0.5)
            self.send_color("blue")
        for i in range(4):
            time.sleep(0.25)
            self.send_color("green")
            time.sleep(0.25)
            self.send_color("red")


    def release_mycobot(self):
        if not self.has_mycobot():
            return
        self.mycobot.release_all_servos()

    def send_color(self, color: str):
        if not self.has_mycobot():
            return

        color_dict = {
            "red": [255, 0, 0],
            "green": [0, 255, 0],
            "blue": [0, 0, 255],
        }
        self.mycobot.set_color(*color_dict[color])
        # print("send color", color)

    # ============================================================
    # Utils method
    # ============================================================
    def has_mycobot(self):
        """Check whether it is connected on mycobot"""
        if not self.mycobot:
            print("no connection to myCobot")
            return False
        return True

    def get_current_time(self):
        """Get current time with format."""
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
        return current_time

if __name__ == "__main__":
    command = sys.argv[1]
    MycobotBTR().run(command)
