import cv2
from cv2 import aruco
import numpy as np
import time
import rospy
import rosservice
import os
import sys
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from rcll_btr_msgs.msg import PictureInfoResponse
from rcll_btr_msgs.srv import PictureInfo
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import numpy as np



def process_image(msg):
    global img
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        # print("get img")
        # cv2.imshow('image', img)
        # cv2.waitKey(1)
    except Exception as err:
        print(err)

def getPicture(data):
    global img, bath_path, n, ext
    # ret, frame = cap.read()
    frame = img
    pictureInfo = PictureInfoResponse()
    pictureInfo.ok = True
    pictureInfo.filename.data = String()
    pictureInfo.filename.data = '{}_{:0>4}.{}'.format(base_path, n, ext)
    cv2.imwrite(pictureInfo.filename.data, frame)
    n += 1
    print(pictureInfo.filename)
    return pictureInfo

def finishCamera():
    global cameraFlag
    cameraFlag = False
    
if __name__ == "__main__":
    args = sys.argv
    topicName = ""
    nodeName = "btr_camera"
    if (len(args) >= 2):
      if ( args[1] == "gazebo" or args[1] == "-g" or args[1] == "--gazebo"):
        topicName = "/robotino" + str(args[2])
        nodeName = "robotino_camera" + str(args[2])

    #  /robotino1/hardware/camera/platform_camera/image_raw/compressed
    rospy.init_node(nodeName)
    cameraFlag = True
    srv01 = rospy.Service(topicName + '/btr_camera/picture', PictureInfo, getPicture)
    rospy.Subscriber(topicName + "/hardware/camera/platform_camera/image_raw", Image, process_image)

    rate = rospy.Rate(10)
    base_path = os.getcwd() + "/../pictures/" + nodeName
    n = 0
    ext = "jpg"
    img = np.full((210, 425, 3), 128, dtype=np.uint8)


    # rospy.spin()
    while not rospy.is_shutdown():
        # if cameraFlag == False:
        #     break
        rate.sleep()
