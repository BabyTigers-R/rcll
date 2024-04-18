import cv2
from cv2 import aruco
import numpy as np
import time
import math
import sys
import rospy
import rosservice
from std_srvs.srv import Empty, EmptyResponse
from rcll_btr_msgs.msg import Corners, TagInfoResponse, PictureInfoResponse, \
                                TagLocationResponse
from rcll_btr_msgs.srv import TagInfo, PictureInfo, TagLocation

def initAruco():
    global dict_aruco, parameters
    # cap = cv2.VideoCapture(0)
    dict_aruco = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()

def getAruco(data):
    global cap, dict_aruco, parameters, corners, topicName
    pictureInfo = PictureInfo()
    rospy.wait_for_service(topicName + '/btr_camera/picture')
    videoCapture = rospy.ServiceProxy(topicName + '/btr_camera/picture', PictureInfo)
    picture = videoCapture()
    print(picture.filename.data)
    
    frame = cv2.imread(picture.filename.data)
    # ret, frame = cap.read()
    # gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    gray = frame
    tagInfo = TagInfoResponse()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dict_aruco, parameters=parameters)
    aruco.drawDetectedMarkers(gray, corners, ids, (0,255,255))
    cv2.imwrite("./test.jpg", gray)
    # print(ids,corners)

    # if (len(ids) == 0):
    if ids is None:
        tagInfo.tag_id.data = 0
        tagInfo.ok = False
    else:
        tagInfo.tag_id.data = int(ids[0])
        tagInfo.UpLeft.x,      tagInfo.UpLeft.y      = corners[0][0][0][0], corners[0][0][0][1]
        tagInfo.UpRight.x,     tagInfo.UpRight.y     = corners[0][0][1][0], corners[0][0][1][1]
        tagInfo.BottomRight.x, tagInfo.BottomRight.y = corners[0][0][2][0], corners[0][0][2][1]
        tagInfo.BottomLeft.x,  tagInfo.BottomLeft.y  = corners[0][0][3][0], corners[0][0][3][1]
        tagInfo.ok = True

    # print(tagInfo)
    return tagInfo

def tagLocation(data):
    global corners, topicName
    tagLocation = TagLocationResponse()
    tagInfo = getAruco(data)
    if (tagInfo.ok == False):
        tagLocation.ok = False
    else:

        marker_length = 0.13

        if (False):
            centerX = (tagInfo.UpLeft.x + tagInfo.UpRight.x + tagInfo.BottomRight.x + tagInfo.BottomLeft.x) / 4
            centerY = (tagInfo.UpLeft.y + tagInfo.UpRight.y + tagInfo.BottomRight.y + tagInfo.BottomLeft.y) / 4

            pictureWidth = 2304
            pictureHeight = 1536

            pictureMoveX = pictureWidth / 2 - centerX
            pictureMoveY = pictureHeight / 2 - centerY

            # =(sqrt((B32-B30)^2+(B33-B31)^2)+sqrt((B34-B36)^2+(B35-B37)^2))/2
            tagSizeX = (math.sqrt((tagInfo.UpRight.x - tagInfo.UpLeft.x) ** 2 + (tagInfo.UpRight.y - tagInfo.UpLeft.y) ** 2) + math.sqrt((tagInfo.BottomRight.x - tagInfo.BottomLeft.x) ** 2 + (tagInfo.BottomRight.y - tagInfo.BottomLeft.y) ** 2)) / 2
            tagSizeY = (math.sqrt((tagInfo.BottomLeft.x - tagInfo.UpLeft.x) ** 2 + (tagInfo.BottomLeft.y - tagInfo.UpLeft.y) ** 2) + math.sqrt((tagInfo.BottomRight.x - tagInfo.UpRight.x) ** 2 + (tagInfo.BottomRight.y - tagInfo.UpRight.y) ** 2)) / 2

            # print("tagSize: ", tagSizeX, tagSizeY)
            if (tagSizeY == 0):
                distanceX = distanceY = 0
                tagLocation.ok = False
            else:
                distanceX = 7.66 * math.exp(-0.0119 * tagSizeY)
                distanceY = marker_length * pictureMoveY / tagSizeY
                tagLocation.ok = True
    
            # print("distance: ", distanceX, distanceY)
            # print(tagLocation)
            tagLocation.tag_location.x = distanceX
            tagLocation.tag_location.y = distanceY
            tagLocation.tag_id.data = tagInfo.tag_id.data
        elif (True):
            # camera_matrix = np.array([[613.17477965,   0.        , 316.44554447],
            #                           [  0.        , 615.60683268, 201.50763039],
            #                           [  0.        ,   0.        ,   1.        ]])
            # distortion_coeff = np.array([ 0.10194539, -0.08779595, -0.0157452 , -0.004555  , -0.19266678])
            # camera for C920 in real world.
            print(topicName)
            if (topicName == "" ):
                camera_matrix = np.array([[614.72443761,   0.        , 329.52316472],
                                          [  0.        , 613.30068366, 199.92578538],
                                          [  0.        ,   0.        ,   1.        ]])
                distortion_coeff = np.array([ 0.10194539, -0.08779595, -0.0157452 , -0.004555  , -0.19266678])
            else:
                # parameter for gazebo
                # K: [715.0546254021291, 0.0, 600.5, 0.0, 715.0546254021291, 350.5, 0.0, 0.0, 1.0]
                camera_matrix = np.array([[715.0546254021291, 0.0, 600.5], 
                                          [0.0, 715.0546254021291, 350.5],
                                          [0.0, 0.0, 1.0]])
                distortion_coeff = np.array([ 0.13505291, -0.29420201, -0.00645303,  0.00196495, -0.01754147])

            minDistance = 1000
            for i, corner in enumerate(corners):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, distortion_coeff)
                # print(i, "corner", corner)
                tvec = np.squeeze(tvec)
                rvec = np.squeeze(rvec)
                rvec_matrix = cv2.Rodrigues(rvec)
                rvec_matrix = rvec_matrix[0] 
                transpose_tvec = tvec[np.newaxis, :].T
                # print(rvec_matrix)
                # print(transpose_tvec)
                proj_matrix = np.hstack((rvec_matrix, transpose_tvec))
  
                euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6] # [deg]
                if (tvec[0] < minDistance):
                    minDistance = tvec[0]
                    print("x : " + str(tvec[0]))
                    print("y : " + str(tvec[1]))
                    print("z : " + str(tvec[2]))

                    print("roll : " + str(euler_angle[0]))
                    # print("pitch: " + str(euler_angle[1]))
                    # print("yaw  : " + str(euler_angle[2]))
                    tagLocation.tag_location.x = tvec[2]
                    tagLocation.tag_location.y = -tvec[0]
                    tagLocation.tag_location.theta = euler_angle[0]
                    tagLocation.tag_id.data = tagInfo.tag_id.data
                    tagLocation.ok = True
    return tagLocation

if __name__ == "__main__":
    args = sys.argv
    topicName = ""
    nodeName = "btr_aruco"
    if (len(args) >= 2):
      if ( args[1] == "gazebo" or args[1] == "-g" or args[1] == "--gazebo"):
        topicName = "/robotino" + str(args[2])
        nodeName = "robotino_aruco" + str(args[2])

    initAruco()
    rospy.init_node(nodeName)
    srv01 = rospy.Service(topicName + '/btr_aruco/TagInfo', TagInfo, getAruco)
    srv02 = rospy.Service(topicName + '/btr_aruco/TagLocation', TagLocation, tagLocation)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
