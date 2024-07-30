#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket 
import time
import threading
import os
import sys
import math
import statistics
import textwrap
import serial
import serial.tools.list_ports

from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD

from ik_solver_myPalletizer import IkSolver 

serialPort = "/dev/ttyAMA0"
#serialBaud = "115200"
serialBaud = "1000000"

class MyPallBTR(object):
    def __init__(self):
        self.mycobot = None
        self.adjust_init = 0
        self.adjust_GR = 14
        # self.adjust_init = -14
        # self.adjust_GR = 0

    def run(self, command, args:None):
        self.connect_mycobot()
        self.mycobot.power_on()
        self.send_color("green")
        print("command: {}".format(command))
        if command == "init":
            self.moveInit()
        if command == "rest":
            self.moveRest()
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
        if command == "moveJoint":
            self.moveJoint(args[0:4], args[4], args[5])
        if command == "moveEnd":
            self.moveEnd(args[0:4], args[4], args[5])
        if command == "moveG":
            self.moveG()
        if command == "moveR":
            self.moveR()
        self.disconnect_mycobot()

    # move to init pose
    def moveInit(self):
        self.mycobot.send_angles([76.2, 47.28, 36.56, 71.98+self.adjust_init], 30)
        time.sleep(1)

    # move to rest pose
    def moveRest(self):
        # self.mycobot.send_angles([89.73, 76.02, -1.75, 73.38], 20)
        self.mycobot.send_angles([92.81, 61.34, -2.46, 117.94+self.adjust_init], 20)
        time.sleep(1)
    
    # run grasping motion with ajustment value (y)
    def moveG(self, y=-80):
        self.moveInit()
        """
        extend = 15 # mm
        list_g = [[270+extend, -80, 55, 90+self.adjust_GR, 15, 0],
                [270+extend, -80, 50, 90+self.adjust_GR, 15, 2],
                [270+extend, -80, 20, 90+self.adjust_GR, 15, 2],
                [270+extend, -80, 5, 90+self.adjust_GR, 15, 2],
                [260+extend, -80, 5, 95+self.adjust_GR, 15, 1],
                [250+extend, -80, 30, 95+self.adjust_GR, 15, 2]]

        for i in range(len(list_g)):
            list_g[i][1] = y
        for i in list_g:
            self.moveEnd(i[0:4], i[4], i[5])
            time.sleep(1)
        """
        list_g = [[50.64, 66.89, -34.54, 80.17+self.adjust_init, 30, 0],
                [0.96, 58.19, -19.59, 29.49+self.adjust_init, 30, 1],
                [5.09, 48.61, 0.52, 28.12+self.adjust_init, 15, 2]]
        for i in list_g:
            self.mycobot.send_angles(i[0:4], i[4])
            time.sleep(1)
            if i[5] == 0:
                self.gripperOpen()
            elif i[5] == 1:
                self.gripperClose()
            else:
                pass
            time.sleep(1)

        self.moveInit()
        self.moveRest()

    # run releaseing motion with ajustment value (y)
    def moveR(self, y=-75):
        self.moveInit()
        extend = 15 # mm
        list_r = [[270+extend, -80, 65, 100+self.adjust_GR, 15, 1],
                [270+extend, -80, 16, 100+self.adjust_GR, 15, 0],
                [270+extend, -80, 16, 90+self.adjust_GR, 15, 2],
                [270+extend, -80, 50, 100+self.adjust_GR, 15, 2]]
    
        for i in range(len(list_r)):
            list_r[i][1] = y
    
        for i in list_r:
            self.moveEnd(i[0:4], i[4], i[5])
            time.sleep(1)
        self.moveInit()
        self.moveRest()
    
    # move arm with end effector's coordinate (x, y, z, omega)
    def moveEnd(self, coordinate, speed, gripper):
        speed = int(speed)
        print("x:", coordinate[0])
        print("y:", coordinate[1])
        print("z:", coordinate[2])
        print("omega:", coordinate[3])
        IKS = IkSolver()
        theta1, theta2, theta3, theta4 = IKS(*coordinate)
        if (theta1 == None) or (theta2 == None) or (theta3 == None) or (theta4 == None):
            print("No angulars found!!!")
            return 0
        self.moveJoint([theta1, theta2, theta3, theta4], speed, gripper)

    # move arm with joint angulars (theta1, theta2, theta3, theta4)
    def moveJoint(self, angulars, speed, gripper):
        self.motor_on()
        speed = int(speed)
        gripper = int(gripper)
        print("speed:", speed)
        print("gripper:", gripper)
        print("angulars:", angulars)
        
        # For a case of no moving
        diff_send_actual = [100, 100, 100, 100]
        while statistics.mean(diff_send_actual) > 2:
            self.mycobot.send_angles(angulars, speed)
            time.sleep(1)
            actual_angulars = self.mycobot.get_angles()
            if len(actual_angulars) == len(angulars):
                diff_send_actual = [abs(angulars[i] - actual_angulars[i]) for i in range(len(angulars))]

        if gripper == 0:
            self.gripperOpen()
        elif gripper == 1:
            self.gripperClose()
        else:
            pass

    def gripperOpen(self):
        # self.mycobot.set_gripper_value(2047, 10)
        self.mycobot.set_gripper_value(255, 30)
        # For a case of no moving
        while (self.mycobot.get_gripper_value() < 70):
            # print(self.mycobot.get_gripper_value())
            if not self.mycobot.is_gripper_moving():
                self.mycobot.set_gripper_value(255, 30)
            time.sleep(2)

    def gripperClose(self):
        # self.mycobot.set_gripper_value(1400, 10)
        self.mycobot.set_gripper_value(0, 30)
        # For a case of no moving
        while (self.mycobot.get_gripper_value() > 70):
            # print(self.mycobot.get_gripper_value())
            if not self.mycobot.is_gripper_moving():
                self.mycobot.set_gripper_value(0, 30)
            time.sleep(2)

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
    if len(sys.argv) < 2:
        MyPallBTR().run(command)
    else:
        args = sys.argv[2:]
        args = list(map(float, args))
        MyPallBTR().run(command, args)
