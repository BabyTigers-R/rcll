# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import sys
import usb
import math

ID_VENDOR_REALSENSE = 0x8086 # Intel
MANUFACTURER_REALSENSE = "Intel(R) RealSense(TM)"
PRODUCT_REALSENSE = "Intel(R) RealSense(TM)"


class module_object_detector():
    def __init__(self, name="ref_image", gazebo=False):
        self.gazebo = gazebo
        if self.gazebo:
            return 0

        # template image
        self.templ = cv2.imread("./templ/belt_template.png", 0)

        self.name = name
        #self.serial_number = "831612071276"
        self.serial_number = "814412071212"
        self.WIDTH = 640
        self.HEIGHT = 480

        # RealSense setting
        self.config = rs.config()
        self.config.enable_device(self.serial_number)
        self.config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 15)
        self.config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 15)

        self.pipeline = rs.pipeline()
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.profile = self.pipeline.start(self.config)

    # A method to detect work pieces position
    def work_detect(self):
        img = self.bg_removed_transparent_work.copy()

        # detect work segements
        contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        center_list_x = []
        center_list_y = []

        # get the center points of the work segments
        for c in contours: 
            # try:
            M = cv2.moments(c)
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            #cv2.circle(self.result, (x, y), 5, (0, 255, 0), thickness=3)
            center_list_x.append(x)
            center_list_y.append(y)
            #except:
            #    return None, None, None

        # if there is no work segment, escape this method.
        if len(center_list_x) <= 0:
            return None, None, None

        # get the centermost pixel regarding of x coordinate.
        elif len(center_list_x) == 1:
            centermost_index = 0

        else:
            centermost_index = center_list_x.index(min(list(map(lambda a: abs(self.WIDTH - a), center_list_x))))

        centermost_x = center_list_x[centermost_index]
        centermost_y = center_list_y[centermost_index]

        d_x, d_y, d_z = self.get_distance(centermost_x, centermost_y)

        # draw circle markers
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for i in range(len(center_list_x)):
            cv2.circle(img, (center_list_x[i], center_list_y[i]), 5, (255, 0, 0), 3)
        cv2.circle(img, (center_list_x[centermost_index], center_list_y[centermost_index]), 5, (0, 255, 0), 3)
        cv2.imwrite('./images/work_detect_result.jpg', img)
        
        return d_x, d_y, d_z

    # A method to detect a conveyer belt position
    def belt_detect(self):
        # 処理対象画像に対して、テンプレート画像との類似度を算出する
        img = self.gray.copy()
        matching_result_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        w = img.shape[1]
        h = img.shape[0]
        res = cv2.matchTemplate(img, self.templ, cv2.TM_CCOEFF_NORMED)

        # 類似度の高い部分を検出する
        # threshold = 0.60
        threshold = 0.56
        # threshold = 0.50
        loc = np.where(res == np.max(res))
        matching_loc = np.array([loc[1][0], loc[0][0]])
        matching_rate = np.max(res)
        print("matching rate: ", matching_rate)

        # 結果を描く
        if matching_rate < threshold:
            cv2.putText(matching_result_img, text='No matching location', org=(0, 25), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.0, color=(0, 0, 255), thickness=2, lineType=cv2.LINE_4)
            cv2.imwrite('./images/belt_matching_result.jpg', matching_result_img)
            return None, None, None

        # テンプレートマッチング画像の高さ、幅を取得する
        h, w = self.templ.shape

        # 結果を描く
        cv2.putText(matching_result_img, text='matching rate: {}'.format(int(matching_rate*1000)/1000), org=(0, 25), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.0, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_4)
        # 検出した部分に緑の○をつける
        x = int(matching_loc[0]) + int(w/2)
        y = int(matching_loc[1]) + int(h/2)
        cv2.circle(matching_result_img, (x, y), 5, (0, 255, 0), 3)
        cv2.imwrite('./images/belt_matching_result.jpg', matching_result_img)

        d_x, d_y, d_z = self.get_distance(x, y)
        return d_x, d_y, d_z

    # A method to get the 3 dimension distance at point (x[pixel], y[pixel])
    def get_distance(self, x, y):
        d_z = self.depth_frame.get_distance(x,y)
        intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        # print(float(intr.width), float(intr.fx))
        d_x = ((x - float(intr.width)/2) * (d_z / float(intr.fx)))
        d_y = ((y - float(intr.height)/2) * (d_z / float(intr.fy)))
        return d_x, d_y, d_z

    def take_photo(self):
        range_min = 0.17
        range_max = 0.25
        bg = 0
        white_color = 255
        work_base_h = (int(50), int(40))
        # camera_y = -115
        # grasping_center_y = -80
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        try:
            frames = self.pipeline.wait_for_frames(30000)
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            self.depth_frame = aligned_frames.get_depth_frame()

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(self.depth_frame.get_data())

            color_image = cv2.resize(color_image, (self.WIDTH, self.HEIGHT))
            depth_image = cv2.resize(depth_image, (640, 480))

            self.gray = cv2.cvtColor(color_image.copy(), cv2.COLOR_BGR2GRAY)
            self.depth_map = depth_image.astype(np.uint8)

            #cv2.imshow('test', self.gray)
            #cv2.waitKey()
            #cv2.destroyAllWindows()
            #cv2.imshow('test', self.depth_map)
            #cv2.waitKey()
            #cv2.destroyAllWindows()

            depth = depth_image.astype(np.float64) * depth_scale

            # make pixel which has more than range_max back ground
            transparent_work = np.where((depth > range_min) & (depth < range_max), white_color, bg)
            self.transparent_work = transparent_work.astype(np.uint8)

            #cv2.imshow('test', self.transparent_work)
            #cv2.waitKey()
            #cv2.destroyAllWindows()

            bg_removed_transparent_work = transparent_work.copy()
            kernel = np.ones((15,15),np.uint8)
            bg_removed_transparent_work = cv2.morphologyEx(bg_removed_transparent_work.astype('uint8'), cv2.MORPH_OPEN, kernel)
            bg_removed_transparent_work = cv2.morphologyEx(bg_removed_transparent_work.astype('uint8'), cv2.MORPH_CLOSE, kernel)

            #bg_removed_transparent_work = cv2.rectangle(bg_removed_transparent_work, (0, 0), (int(bg_removed_transparent_work.shape[1]), int(bg_removed_transparent_work.shape[0]/2)+work_base_h), 255, thickness=-1)
            bg_removed_transparent_work = cv2.rectangle(bg_removed_transparent_work, (0, 0), (int(bg_removed_transparent_work.shape[1]), int(bg_removed_transparent_work.shape[0]/2)-work_base_h[0]), bg, thickness=-1)
            self.bg_removed_transparent_work = cv2.rectangle(bg_removed_transparent_work, (0, int(bg_removed_transparent_work.shape[0]/2)-work_base_h[1]), (bg_removed_transparent_work.shape[1], bg_removed_transparent_work.shape[0]), bg, thickness=-1)

            cv2.imwrite('./images/{}_gray.jpg'.format(self.name), self.gray)
            cv2.imwrite('./images/{}_depth_map.jpg'.format(self.name), self.depth_map)
            cv2.imwrite('./images/{}_transparent_work.jpg'.format(self.name), self.transparent_work)
            cv2.imwrite('./images/{}_bg_removed_transparent_work.jpg'.format(self.name), self.bg_removed_transparent_work)

            return 1

        except KeyboardInterrupt:
            return 0

def main():
    inst = module_object_detector()
    time.sleep(3)
    inst.take_photo()
    print(inst.belt_detect())
    print(inst.work_detect())
    inst.pipeline.stop()

if __name__ == "__main__":
    main()
