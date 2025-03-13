import cv2
from cv2 import aruco
import numpy as np
import time
# import rospy
import rclpy
from rclpy.node import Node
# import rosservice
import os
# from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from btr2_msgs.srv import PictureInfo

class btr2_camera(Node):
  def __init__(self):
    self.initCamera()
    # rospy.init_node('btr_camera')
    super().__init__('btr2_camera')
    self.cameraFlag = True
    self.srv01 = self.create_service(PictureInfo, 'btr2_camera/picture', self.getPicture)
    self.base_path = os.getcwd() + "/../pictures/btr2-camera"
    self.n = 0
    self.ext = "jpg"

    # timer_period = 1
    # self.timer = self.create_timer(timer_period, self.timer_callback)

  def initCamera(self):
    # global cap
    self.cap = cv2.VideoCapture(0 + 6)
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

  def getPicture(self, request, response):
    # global cap, bath_path, n, ext
    ret, frame = self.cap.read()
    # pictureInfo = PictureInfoResponse()
    # pictureInfo.ok = ret
    response.ok = ret
    # pictureInfo.filename.data = String()
    # pictureInfo.filename.data = '{}_{:0>4}.{}'.format(self.base_path, self.n, self.ext)
    filename = String()
    filename = '{}_{:0>4}.{}'.format(self.base_path, self.n, self.ext)
    cv2.imwrite(filename, frame)
    self.n += 1
    response.filename.data = filename
    self.get_logger().info(response.filename.data)
    return response

  def finishCamera(self):
    # global cap, cameraFlag
    self.cameraFlag = False
    
def main(args=None):
  rclpy.init(args=args)
  btr2 = btr2_camera()
  rclpy.spin(btr2)
  rclpy.shutdown()

if __name__ == "__main__":
  main()
