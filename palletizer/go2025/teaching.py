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
        #if command == "init":
        #    self.gripperClose()
        #    self.mycobot.send_angles([0, 0, 0, 0], 30)
        if command == "status":
            self.getStatus()
        if command == "teach":
            self.teaching()
        if command == "open":
            self.gripperOpen()
        if command == "close":
            self.gripperClose()
        # if command == "gripper":
        #    print(self.mycobot.get_gripper_value(), self.mycobot.is_gripper_moving())
        self.disconnect_mycobot()

    def gripperOpen(self):
        try:
            self.mycobot.set_gripper_value(255, 30)
        except Exception as e:
            print(e)
            pass
        time.sleep(0.5)

    def gripperClose(self):
        try:
            self.mycobot.set_gripper_value(0, 30)
        except Exception as e:
            print(e)
            pass
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
        print("Angles: ", "[{} {} {} {}]".format(*self.mycobot.get_angles()))
        # print("Radians: ", self.mycobot.get_radians())

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
