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
import math

from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD

serialPort = "/dev/ttyAMA0"
# serialBaud = "115200"
serialBaud = "1000000"

class MycobotBTR(object):
    def __init__(self):
        self.mycobot = None
        self.angle_tolerance = 3

    def run(self, mode:str, position:list):
        self.connect_mycobot()
        self.mycobot.power_on()
        self.send_color("green")

        f = open("/home/er/git/rcll/palletizer/motion/{}.txt".format(mode), 'r')
        self.data = f.read()
        self.moveArm([-75, 0, 15, -45], 40)
        self.moveArm([0, 0, 0, -45], 60)
        self.playback(position)
        self.moveArm([0, 0, 0, -45], 60)
        self.moveArm([-75, 0, 15, -45], 40)
        f.close()
        
        self.disconnect_mycobot()


    def playback(self, position):
        default_length = 0.2175
        command_list = ["move", "wait", "close", "open"]
        splited_data = self.data.split()
        l = len(splited_data)
        # pointer to read command data
        p = 0

        while p < l:
            # move
            if splited_data[p] == command_list[0]:
                # adjust the angles
                position_x = float(position[1])
                position_y = -float(position[0])
                if (position_x == 0) and (position_y == 0):
                    goal_angles = [float(splited_data[p+1]), float(splited_data[p+2]), float(splited_data[p+3]), float(splited_data[p+4])]
                    self.moveArm(goal_angles, int(float(splited_data[p+5])))
                    p += 6
                
                else:
                    j1 = math.degrees(math.atan2(position_y, -position_x))
                    j4 = j1
                    link_length = math.sqrt(position_x**2 + position_y**2)
                    j2_adjust = ((link_length / default_length) - 1) * 110 # 110 is a parameter to adjust j2
                    j3_adjust = ((link_length / default_length) - 1) * 190 # 190 is a parameter to adjust j3

                    goal_angles = [float(splited_data[p+1]) + j1, float(splited_data[p+2]) + j2_adjust, float(splited_data[p+3]) - j3_adjust, float(splited_data[p+4]) + j4]
                    self.moveArm(goal_angles, int(float(splited_data[p+5])))
                    p += 6
            
            # wait
            elif splited_data[p] == command_list[1]:
                time.sleep(float(splited_data[p+1]))
                p += 2
            
            # close
            elif splited_data[p] == command_list[2]:
                self.gripperClose()
                self.getStatus()
                p += 1
            
            # open
            elif splited_data[p] == command_list[3]:
                self.gripperOpen()
                self.getStatus()
                p += 1

            else:
                p += 1


    def moveArm(self, goal_angles:list, speed):
        self.mycobot.send_angles(goal_angles, speed)

        # check current angles
        start_time = time.time()
        self.getStatus()
        errors = [goal_angles[i] - self.angles[i] for i in range(4)]
        print("Angle errors: ", errors)
        while (abs(errors[0]) > self.angle_tolerance) or (abs(errors[1]) > self.angle_tolerance) or (abs(errors[2]) > self.angle_tolerance)  or (abs(errors[3]) > self.angle_tolerance):
            self.getStatus()
            past_time = time.time() - start_time
            print(past_time)
            if past_time > 5:
                break
            errors = [goal_angles[i] - self.angles[i] for i in range(4)]
            print("Angle errors: ", errors)

    def gripperOpen(self):
        try:
            self.mycobot.set_gripper_value(20, 30)
            time.sleep(2)
            self.mycobot.set_gripper_value(60, 30)
            time.sleep(2)
            self.mycobot.set_gripper_value(100, 30)
            time.sleep(1)
            self.getStatus()
            start_time = time.time()
            while self.gripper_value < 70:
                self.getStatus()
                past_time = time.time() - start_time
                print(past_time)
                if past_time > 5:
                    break
        except Exception as e:
            print(e)
            pass

    def gripperClose(self):
        try:
            self.mycobot.set_gripper_value(80, 30)
            time.sleep(2)
            self.mycobot.set_gripper_value(40, 30)
            time.sleep(2)
            self.mycobot.set_gripper_value(0, 30)
            time.sleep(1)
            self.getStatus()
            start_time = time.time()
            while self.gripper_value > 30:
                self.getStatus()
                past_time = time.time() - start_time
                print(past_time)
                if past_time > 5:
                    break
        except Exception as e:
            print(e)
            pass

    def getStatus(self):
        angles = [*self.mycobot.get_angles()]
        while len(angles) != 4:
            angles = [*self.mycobot.get_angles()]
        self.angles = angles
        self.gripper_value = self.mycobot.get_gripper_value()
        print("Angles: ", "{}".format(self.angles))
        print("Gripper value: ", "{}".format(self.gripper_value))

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
    mode = str(sys.argv[1])
    # position = [0.05, 0.23]
    # position = [0.05, 0.2175]
    # position = [0.08609624728956461, 0.223000004887580870]
    # position = [0.0, 0.2175]
    position = [float(sys.argv[2]), float(sys.argv[3])]
    MycobotBTR().run(mode, position)
