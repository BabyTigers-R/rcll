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

    def run(self, mode:str):
        self.connect_mycobot()
        self.mycobot.power_on()
        self.send_color("green")

        f = open("/home/er/yasuda_pall/jo2025/motion/{}.txt".format(mode), 'r')
        self.data = f.read()
        self.move()
        f.close()

        self.disconnect_mycobot()

    def move(self):
        command_list = ["move", "wait", "close", "open"]
        splited_data = self.data.split()
        l = len(splited_data)
        p = 0
        while p < l:
            if splited_data[p] == command_list[0]:
                self.mycobot.send_angles([float(splited_data[p+1]), float(splited_data[p+2]), float(splited_data[p+3]), float(splited_data[p+4])], int(float(splited_data[p+5])))
                p += 6
            elif splited_data[p] == command_list[1]:
                time.sleep(float(splited_data[p+1]))
                p += 2
            elif splited_data[p] == command_list[2]:
                self.gripperClose()
                p += 1
            elif splited_data[p] == command_list[3]:
                self.gripperOpen()
                p += 1


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

    def getStatus(self):
        print("Angles: ", self.mycobot.get_angles())
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
    mode = str(sys.argv[1])
    MycobotBTR().run(mode)
