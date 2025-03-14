#!/usr/bin/python
import sys
import subprocess
# import rospy
import rclpy
from rclpy.node import Node
import btr2_refbox
from btr2_refbox import refbox
import btr2_rcll2025

class btr2_game2025(Node):
  def __init__(self):
    self.gazeboFlag = True
    self.robotNum = 1
    self.topicName = "/robot" + str(self.robotNum)
    self.nodeName = "btr_2025_" + str(self.robotNum)
    self.teamName = "BabyTigers-W"
    self.challenge = "test"
    self.gazeboFlag = False

    self.refbox =     btr2_refbox.refbox(teamName = self.teamName, robotNum = self.robotNum, gazeboFlag = self.gazeboFlag)
    print(self.refbox)
    self.rcll   = btr2_rcll2025.btr_rcll(teamName = self.teamName, robotNum = self.robotNum, gazeboFlag = self.gazeboFlag, refbox = self.refbox)

    print(self.challenge)
    self.challengeFlag = True
    
    while True:
        rclpy.spin_once(self.refbox)
        print("game2025: ", self.challenge, self.refbox.refboxGamePhase) 
        self.refbox.sendBeacon()

def main(args=None):
  rclpy.init(args=args)
  btr2 = btr2_game2025()
  rclpy.spin(btr2)
  rclpy.shutdown()

if __name__ == "__main__":
  main()

  print("game2024: ", challenge, refbox.refboxGamePhase)
  if (challengeFlag):
      if (oldGamePhase != refbox.refboxGamePhase):
          print("refboxGamePhase: ", refbox.refboxGamePhase)
          oldGamePhase = refbox.refboxGamePhase
          if (challenge == "exploration" and refbox.refboxGamePhase == 20 ):
              rcll.challenge("exploration")
              challengeFlag = False
          elif (challenge == "grasping" and refbox.refboxGamePhase == 30):
              # elif (challenge == "grasping"):
              print(refbox.refboxGamePhase)
              rcll.challenge("grasping")
              challengeFlag = False
          elif (challenge == "graspingTest"):
              print(refbox.refboxGamePhase)
              rcll.challenge("graspingTest")
              challengeFlag = False
          elif (challenge == "navigation" and refbox.refboxGamePhase == 30):
              rcll.challenge("navigation")
              challengeFlag = False
          elif (challenge == "production" and refbox.refboxGamePhase == 30):
              rcll.challenge("production")
              challengeFlag = False
          elif (challenge == "productionTest"):
              rcll.challenge("production")
          elif (challenge == "beacon"):
              refbox.sendBeacon()
              print("Game status is ", refbox.refboxGamePhase)
          if (challenge == "gazebo"):
              print("Game status is ", refbox.refboxGamePhase)
              rcll.challenge("turn")
              challengeFlag = False
                
              if (refbox.refboxGamePhase == 10):
                  rcll.challenge("reset1")
              if (refbox.refboxGamePhase == 20):
                  rcll.challenge("main_exploration")
              if (refbox.refboxGamePhase == 30):
                  rcll.challenge("production")
      # print(refbox.refboxGamePhase)

      refbox.sendBeacon()
      rate.sleep()


