# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
import sys
import rospy
import rosservice
from std_srvs.srv import Empty, EmptyResponse
from rcll_btr_msgs.msg import Corners, TagInfoResponse, PictureInfoResponse, \
                                TagLocationResponse
from rcll_btr_msgs.srv import TagInfo, PictureInfo, TagLocation

class module_photographer_by_c920():
    def __init__(self, name, gazebo=False):
        self.gazebo = gazebo
        self.name = name

    def run(self):
        if self.gazebo:
            return 0
        #cap = cv2.VideoCapture(0)

        pictureInfo = PictureInfo()
        rospy.wait_for_service('/btr_camera/picture')
        videoCapture = rospy.ServiceProxy('/btr_camera/picture', PictureInfo)
        picture = videoCapture()
        frame = cv2.imread(picture.filename.data)

        #time.sleep(1)
        #ret, frame = cap.read()
        ret = True
        
        if ret:
            gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.WIDTH = gray_image.shape[1]
            self.HEIGHT = gray_image.shape[0]
        
            cv2.imwrite('./images/{}.jpg'.format(self.name), gray_image)
            
            #cap.release()
            cv2.destroyAllWindows()

        else:
            #cap.release()
            cv2.destroyAllWindows()

def main():
    #jpg画像の名前
    if len(sys.argv) > 1:
        name = sys.argv[1]
    else:
        print("Error! Please set args!!!")
        quit()

    inst = module_photographer_by_c920(name)
    inst.run()

if __name__ == "__main__":
    main()
