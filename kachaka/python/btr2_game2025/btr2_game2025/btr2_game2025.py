#!/usr/bin/python
import sys
import subprocess
# import rospy
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
import btr2_refbox
from btr2_refbox import refbox
import btr2_rcll2025
from std_msgs.msg import Int8

class btr2_game2025(Node):
  def __init__(self):
    super().__init__('btr2_game2025')
    self.gazeboFlag = True
    self.robotNum = 1
    self.topicName = "/robot" + str(self.robotNum)
    self.nodeName = "btr_2025_" + str(self.robotNum)
    self.teamName = "BabyTigers-W"
    self.challenge = "navigation"
    self.gazeboFlag = False

    self.refbox =     btr2_refbox.refbox(teamName = self.teamName, robotNum = self.robotNum, gazeboFlag = self.gazeboFlag, challenge = self.challenge)
    print(self.refbox)
    self.rcll   = btr2_rcll2025.btr_rcll(teamName = self.teamName, robotNum = self.robotNum, gazeboFlag = self.gazeboFlag, refbox = self.refbox)

    print(self.challenge)
    self.challengeFlag = True
    self.oldGamePhase = Int8()  # self.refbox.refboxGamePhase

def main(args=None):
  rclpy.init(args=args)
  my_executor = MultiThreadedExecutor()
  btr2 = btr2_game2025()
  # rclpy.spin(btr2)
  my_executor.add_node(btr2)
  my_executor.add_node(btr2.refbox)
  my_executor.add_node(btr2.rcll)

  while True:
    # rclpy.spin_once(self.refbox)
    my_executor.spin_once()
    print("game2025: ", btr2.challenge, btr2.refbox.refboxGamePhase)
    btr2.refbox.sendBeacon()

    if (btr2.challengeFlag):
        if (btr2.oldGamePhase != btr2.refbox.refboxGamePhase):
            print("refboxGamePhase: ", btr2.refbox.refboxGamePhase)
            btr2.oldGamePhase = btr2.refbox.refboxGamePhase
            if (btr2.challenge == "exploration" and btr2.refbox.refboxGamePhase == 20 ):
                btr2.rcll.challenge("exploration")
                btr2.challengeFlag = False
            elif (btr2.challenge == "grasping" and btr2.refbox.refboxGamePhase == 30):
                btr2.rcll.challenge("grasping")
                btr2.challengeFlag = False
            elif (btr2.challenge == "navigation" and btr2.refbox.refboxGamePhase == 30):
                btr2.rcll.challenge("navigation")
                btr2.challengeFlag = False
            elif (btr2.challenge == "production" and btr2.refbox.refboxGamePhase == 30):
                btr2.rcll.challenge("production")
                btr2.challengeFlag = False
            elif (btr2.challenge == "beacon"):
                btr2.refbox.sendBeacon()
            elif (btr2.challenge == "gazebo"):
                btr2.rcll.challenge("turn")
                btr2.challengeFlag = False
                if (btr2.refbox.refboxGamePhase == 10):
                    btr2.rcll.challenge("reset1")
                if (btr2.refbox.refboxGamePhase == 20):
                    btr2.rcll.challenge("main_exploration")
                if (btr2.refbox.refboxGamePhase == 30):
                    btr2.rcll.challenge("production")


  my_executor.shutdonw()
  btr2.rcll.destory_ndoe()
  btr2.refbox.destory_node()
  btr2.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()
