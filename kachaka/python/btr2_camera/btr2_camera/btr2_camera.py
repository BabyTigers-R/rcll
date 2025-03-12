import cv2
from cv2 import aruco
import numpy as np
import time
# import rospy
import rclpy
import rosservice
import os
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from btr2_msgs.msg import PictureInfoResponse
from btr2_msgs.srv import PictureInfo

class btr_camera(Node):
  def __init__(self):
    self.initCamera()
    # rospy.init_node('btr_camera')
    super().__init__('btr2_camera')
    self.cameraFlag = True
    self.srv01 = rpcpy.create_service(PictureInfo, 'btr2_camera/picture', getPicture)
    self.base_path = os.getcwd() + "/../pictures/btr2-camera"
    self.n = 0
    self.ext = "jpg"

    # timer_period = 1
    # self.timer = self.create_timer(timer_period, self.timer_callback)

  def initCamera(self):
    # global cap
    self.cap = cv2.VideoCapture(0)
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

  def getPicture(self, data):
    # global cap, bath_path, n, ext
    ret, frame = self.cap.read()
    pictureInfo = PictureInfoResponse()
    pictureInfo.ok = ret
    pictureInfo.filename.data = String()
    pictureInfo.filename.data = '{}_{:0>4}.{}'.format(self.base_path, self.n, self.ext)
    cv2.imwrite(pictureInfo.filename.data, frame)
    self.n += 1
    print(pictureInfo.filename)
    return pictureInfo

  def finishCamera(self):
    # global cap, cameraFlag
    self.cameraFlag = False
    
def main(args=None):
  rclpy.init(args=args)
  btr2_camera = btr2_camera()
  rclpy.spin(btr2_camera)
  rclpy.shutdown()

if __name__ == "__main__":
  main()
