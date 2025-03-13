import cv2
from cv2 import aruco
import numpy as np
import time
import math
import sys
# import rospy
import rclpy
from rclpy.node import Node
# import rosservice
# from std_srvs.srv import Empty, EmptyResponse
import std_msgs
# from btr2_msgs.msg import Corners, TagInfoResponse, PictureInfoResponse, \
#                                 TagLocationResponse
from btr2_msgs.msg import Corners
from btr2_msgs.srv import TagInfo, PictureInfo, TagLocation
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2.aruco as aruco
import numpy as np
import os

from rclpy.executors import MultiThreadedExecutor

# ArUcoマーカー
ArUco_number = {
    "101": [[0,1,1,1,1],[0,1,0,0,0],[1,0,1,1,0],[0,1,0,0,0],[0,1,0,0,0]],
    "102": [[0,1,1,1,1],[0,1,0,0,0],[1,0,1,1,0],[0,1,0,0,0],[1,0,1,1,0]],
    "103": [[0,1,1,1,1],[0,1,0,0,0],[1,0,1,1,0],[0,1,0,0,0],[1,0,0,0,1]],
    "104": [[0,1,1,1,1],[0,1,0,0,0],[1,0,1,1,0],[1,0,1,1,0],[0,1,1,1,1]],
    "111": [[0,1,1,1,1],[0,1,0,0,0],[1,0,1,1,0],[1,0,0,0,1],[1,0,0,0,1]],
    "112": [[0,1,1,1,1],[0,1,0,0,0],[1,0,0,0,1],[0,1,1,1,1],[0,1,1,1,1]],
    "113": [[0,1,1,1,1],[0,1,0,0,0],[1,0,0,0,1],[0,1,1,1,1],[0,1,0,0,0]],
    "114": [[0,1,1,1,1],[0,1,0,0,0],[1,0,0,0,1],[0,1,1,1,1],[1,0,1,1,0]],
    "121": [[0,1,1,1,1],[0,1,0,0,0],[1,0,0,0,1],[1,0,1,1,0],[0,1,0,0,0]],
    "122": [[0,1,1,1,1],[0,1,0,0,0],[1,0,0,0,1],[1,0,1,1,0],[1,0,1,1,0]],
    "131": [[0,1,1,1,1],[1,0,1,1,0],[0,1,1,1,1],[0,1,1,1,1],[1,0,0,0,1]],
    "132": [[0,1,1,1,1],[1,0,1,1,0],[0,1,1,1,1],[0,1,0,0,0],[0,1,1,1,1]],
    "141": [[0,1,1,1,1],[1,0,1,1,0],[0,1,1,1,1],[1,0,0,0,1],[0,1,0,0,0]],
    "142": [[0,1,1,1,1],[1,0,1,1,0],[0,1,1,1,1],[1,0,0,0,1],[1,0,1,1,0]],
    "201": [[0,1,1,1,1],[1,0,0,0,1],[0,1,1,1,1],[1,0,1,1,0],[0,1,0,0,0]],
    "202": [[0,1,1,1,1],[1,0,0,0,1],[0,1,1,1,1],[1,0,1,1,0],[1,0,1,1,0]],
    "203": [[0,1,1,1,1],[1,0,0,0,1],[0,1,1,1,1],[1,0,1,1,0],[1,0,0,0,1]],
    "204": [[0,1,1,1,1],[1,0,0,0,1],[0,1,1,1,1],[1,0,0,0,1],[0,1,1,1,1]],
    "211": [[0,1,1,1,1],[1,0,0,0,1],[0,1,0,0,0],[0,1,1,1,1],[1,0,0,0,1]],
    "212": [[0,1,1,1,1],[1,0,0,0,1],[0,1,0,0,0],[0,1,0,0,0],[0,1,1,1,1]],
    "213": [[0,1,1,1,1],[1,0,0,0,1],[0,1,0,0,0],[0,1,0,0,0],[0,1,0,0,0]],
    "214": [[0,1,1,1,1],[1,0,0,0,1],[0,1,0,0,0],[0,1,0,0,0],[1,0,1,1,0]],
    "221": [[0,1,1,1,1],[1,0,0,0,1],[0,1,0,0,0],[1,0,0,0,1],[0,1,0,0,0]],
    "222": [[0,1,1,1,1],[1,0,0,0,1],[0,1,0,0,0],[1,0,0,0,1],[1,0,1,1,0]],
    "231": [[0,1,1,1,1],[1,0,0,0,1],[1,0,1,1,0],[0,1,0,0,0],[1,0,0,0,1]],
    "232": [[0,1,1,1,1],[1,0,0,0,1],[1,0,1,1,0],[1,0,1,1,0],[0,1,1,1,1]],
    "241": [[0,1,1,1,1],[1,0,0,0,1],[1,0,0,0,1],[0,1,1,1,1],[0,1,0,0,0]],
    "242": [[0,1,1,1,1],[1,0,0,0,1],[1,0,0,0,1],[0,1,1,1,1],[1,0,1,1,0]],
    "301": [[0,1,0,0,0],[0,1,1,1,1],[1,0,1,1,0],[1,0,0,0,1],[0,1,0,0,0]],
    "302": [[0,1,0,0,0],[0,1,1,1,1],[1,0,1,1,0],[1,0,0,0,1],[1,0,1,1,0]]
}



# __________________________________________________
# 追加関数
# パターンの定義
patterns = [
    [0, 1, 0, 0, 0],  # 01000
    [0, 1, 1, 1, 1],  # 01111
    [1, 0, 0, 0, 1],  # 10001
    [1, 0, 1, 1, 0]   # 10110
]

class btr2_aruco(Node):
  def __init__(self):
    # super().__init__('btr2_aruco')
    self.topicName = ""
    nodeName = "btr2_aruco"
    # if (len(args) >= 2):
    #  if ( args[1] == "gazebo" or args[1] == "-g" or args[1] == "--gazebo"):
    #    self.topicName = "/kachaka" + str(args[2])
    #    nodeName = "kachaka_aruco" + str(args[2])

    self.initAruco()
    super().__init__(nodeName)
    self.srv01 = self.create_service(TagInfo, self.topicName + '/btr2_aruco/TagInfo', self.getAruco)
    self.srv02 = self.create_service(TagLocation, self.topicName + '/btr2_aruco/TagLocation', self.tagLocation)

  def process_image_7_7(self, image):
    # 画像をグレースケールに変換
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 二値化
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    
    height, width = binary.shape
    grid = []
    # マス目ごとに処理
    for i in range(7):  # 外側のマスを省くために1から始める
        row = []
        for j in range(7):  # 外側のマスを省くために1から始める
            # マスの範囲を計算
            y_start = height // 7 * i
            y_end = height // 7 * (i + 1)
            x_start = width // 7 * j
            x_end = width // 7 * (j + 1)
            # マスの領域を切り出し
            cell = binary[y_start:y_end, x_start:x_end]
            # マスの中身が60%以上が黒かどうかを判定
            black_pixels = np.count_nonzero(cell == 0)
            if black_pixels / cell.size >= 0.6:  # 60%以上が黒なら1
                row.append(1)
            else:
                row.append(0)
        grid.append(row)
    
    del grid[0]
    del grid[5]
    del grid[0][0]
    del grid[0][5]

    del grid[1][0]
    del grid[1][5]

    del grid[2][0]
    del grid[2][5]

    del grid[3][0]
    del grid[3][5]

    del grid[4][0]
    del grid[4][5]

    # ハミング距離の計算と訂正
    corrected_grid = process_grid(grid, patterns)
    print(corrected_grid)
    grid_count = 0

    if corrected_grid == [[0,1,1,1,1],[0,1,0,0,0],[1,0,1,1,0],[0,1,0,0,0],[0,1,0,0,0]]:
        grid_count += 1
    
    # return grid, grid_count, corrected_grid
    return grid,corrected_grid


  def hamming_distance(self, pattern1, pattern2):
    """
    ハミング距離を計算する関数
    """
    # print(zip(pattern1, pattern2))
    return sum(bit[0] != bit[1] for bit in map(list, zip(pattern1, pattern2)))

  def find_nearest_pattern(self, patterns, target_pattern):
    """
    最も近いパターンを探索する関数
    """
    min_distance = float('inf')
    nearest_pattern = None
    for pattern in patterns:
        distance = hamming_distance(pattern, target_pattern)
        if distance < min_distance:
            min_distance = distance
            nearest_pattern = pattern
    return nearest_pattern

  def process_grid(self, grid, patterns):
    """
    パターンとの比較および最も近いパターンの検出を行う関数
    """
    corrected_grid = []
    for row in grid:
        # corrected_row = []
        # for pattern in row:
        nearest_pattern = find_nearest_pattern(patterns, row)
        corrected_grid.append(nearest_pattern)
    # corrected_grid.append(corrected_row)
    return corrected_grid

# __________________________________________________


  def initAruco(self):
    # global dict_aruco, parameters
    # cap = cv2.VideoCapture(0)
    self.dict_aruco = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    self.dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    self.parameters = aruco.DetectorParameters_create()

  def getAruco(self, request, response):
    print("called getAruco")
    # global cap, dict_aruco, parameters, corners, topicName
    # pictureInfo = PictureInfo()
    # rclpy.wait_for_service(topicName + '/btr2_camera/picture')
    videoCapture = request.filename.data
    print(videoCapture)
    
    # frame = cv2.imread(picture.filename.data)
    frame = cv2.imread(videoCapture)
    # ret, frame = self.cap.read()
    # gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    gray = frame
    # tagInfo = TagInfo_Response()
    tagInfo = response
    
    self.corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.dict_aruco, parameters=self.parameters)
    aruco.drawDetectedMarkers(gray, self.corners, ids, (0,255,255))
    cv2.imwrite("./test.jpg", gray)
    # print(ids, self.corners)

    # if (len(ids) == 0):
    if ids is None:
        tagInfo.tag_id.data = 0
        tagInfo.ok = False
    else:
        tagInfo.tag_id.data = int(ids[0])
        tagInfo.UpLeft.x,      tagInfo.UpLeft.y      = self.corners[0][0][0][0], self.corners[0][0][0][1]
        tagInfo.UpRight.x,     tagInfo.UpRight.y     = self.corners[0][0][1][0], self.corners[0][0][1][1]
        tagInfo.BottomRight.x, tagInfo.BottomRight.y = self.corners[0][0][2][0], self.corners[0][0][2][1]
        tagInfo.BottomLeft.x,  tagInfo.BottomLeft.y  = self.corners[0][0][3][0], self.corners[0][0][3][1]
        tagInfo.ok = True

        
        # ________________________________________________________________________________________
        # 追加
        # ここに，上記の４隅についての座標使ってあるArUcoのプログラムを入れる．
        # 画像をグレースケールに変換
        # Initialize variables with initial values
        max_x = tagInfo.UpLeft.x
        min_x = tagInfo.UpLeft.x
        max_y = tagInfo.UpLeft.y
        min_y = tagInfo.UpLeft.y

        # Update variables by comparing with other points
        # Update for UpRight
        max_x = max(max_x, tagInfo.UpRight.x)
        min_x = min(min_x, tagInfo.UpRight.x)
        max_y = max(max_y, tagInfo.UpRight.y)
        min_y = min(min_y, tagInfo.UpRight.y)

        # Update for BottomRight
        max_x = max(max_x, tagInfo.BottomRight.x)
        min_x = min(min_x, tagInfo.BottomRight.x)
        max_y = max(max_y, tagInfo.BottomRight.y)
        min_y = min(min_y, tagInfo.BottomRight.y)

        # Update for BottomLeft
        max_x = max(max_x, tagInfo.BottomLeft.x)
        min_x = min(min_x, tagInfo.BottomLeft.x)
        max_y = max(max_y, tagInfo.BottomLeft.y)
        min_y = min(min_y, tagInfo.BottomLeft.y)

        # # Output the results
        # print("Max X:", max_x)
        # print("Min X:", min_x)
        # print("Max Y:", max_y)
        # print("Min Y:", min_y)

        img2 = gray[int(min_y):int(max_y), int(min_x):int(max_x)] # マーカー部分を切り取る
        # 画像を7x7のマスに分割して、各マスの中身が黒なら1、それ以外は0とする
        grid, corrected_grid = process_image_7_7(img2)


        # ArUco_numberとマッチするタグの番号を格納するリスト
        matched_tags = []
        # ArUco_numberの辞書と比較する
        matched_tags_int = 0
        for tag, pattern in ArUco_number.items():
            if pattern == corrected_grid:
                matched_tags.append(tag)
                matched_tags_int = int(matched_tags[0])
        # マッチしたタグの番号を出力する
        print("Matched tags:", matched_tags)

 
        tagInfo.tag_id.data = matched_tags_int  #int(ids[0])
        # ____________________________________________________________________________________________

    # response.tag_id = std_msgs.msg.Int16(data=tagInfo.tag_id.data) 
    tagInfo.error_msg = ""
    print(tagInfo)
    return tagInfo

  def tagLocation(self, request, response):
    # global corners, topicName
    # tagLocation = TagLocationResponse()
    tagLocation = response
    tagInfo = self.getAruco(request, response)
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
            print(self.topicName)
            if (self.topicName == "" ):
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
            for i, corner in enumerate(self.corners):
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
    tagLocation.error_msg = ""
    print(tagLocation)
    return tagLocation

def main(args=None):
  rclpy.init(args=args)
  btr2 = btr2_aruco()

  rclpy.spin(btr2)
  rclpy.shutdown()

if __name__ == "__main__":
  main()

