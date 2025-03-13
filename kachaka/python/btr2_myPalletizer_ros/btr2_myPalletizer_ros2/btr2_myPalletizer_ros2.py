#!/usr/bin/env python
import os
# import rospy
import rclpy
from rclpy.node import Node
import sys
from std_srvs.srv import Empty
CMD = "ssh palletizer-0 -l er -i /home/ryukoku/.ssh/id_rsa_pall python3 /home/er/yasuda_pall/eindhoven2024/MyPallBTR.py"

class btr2_myPalletizer(Node):
  def __init__(self):
    super().__init__('btr2_myPalletizer_ros')
    self.srv01 = self.create_service(Empty, '/btr2_arm/move_g', self.grab_arm)
    self.srv02 = self.create_service(Empty, '/btr2_arm/move_r', self.release_arm)

  def grab_arm(self, request, response):
    cmd = CMD + " moveG"
    print(cmd)
    os.system(cmd)
    return response

  def release_arm(self, request, response):
    cmd = CMD + " moveR"
    print(cmd)
    os.system(cmd)
    return response

def main(args=None):
    rclpy.init(args=args)
    btr2 = btr2_myPalletizer()
    rclpy.spin(btr2)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
