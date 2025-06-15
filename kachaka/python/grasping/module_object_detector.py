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
        self.name = name
        self.gazebo = gazebo
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
        time.sleep(3)

    # A method to detect work pieces position
    def work_detect(self):
        x = 0
        y = 0
        return x, y

    # A method to detect a conveyer belt position
    def belt_detect(self):
        x = 0
        y = 0
        return x, y

    def take_photo(self, belt=False):
        if self.gazebo:
            return 0
        #time.sleep(3)
        range_min = 0.17
        range_max = 0.23
        bg = 0
        white_color = 255
        work_base_h = (int(50), int(40))
        camera_y = -115
        grasping_center_y = -80
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

            gray = cv2.cvtColor(color_image.copy(), cv2.COLOR_BGR2GRAY)
            depth_map = depth_image.astype(np.uint8)

            cv2.imshow('test', gray)
            cv2.waitKey()
            cv2.destroyAllWindows()
            cv2.imshow('test', depth_map)
            cv2.waitKey()
            cv2.destroyAllWindows()

            depth = depth_image.astype(np.float64) * depth_scale

            # make pixel which has more than range_max back ground
            transparent_work = np.where((depth > range_min) & (depth < range_max), white_color, bg)
            transparent_work = transparent_work.astype(np.uint8)

            cv2.imshow('test', transparent_work)
            cv2.waitKey()
            cv2.destroyAllWindows()

            bg_removed_transparent_work = transparent_work.copy()
            kernel = np.ones((15,15),np.uint8)
            bg_removed_transparent_work = cv2.morphologyEx(bg_removed_transparent_work, cv2.MORPH_OPEN, kernel)
            bg_removed_transparent_work = cv2.morphologyEx(bg_removed_transparent_work, cv2.MORPH_CLOSE, kernel)

            #bg_removed_transparent_work = cv2.rectangle(bg_removed_transparent_work, (0, 0), (int(bg_removed_transparent_work.shape[1]), int(bg_removed_transparent_work.shape[0]/2)+work_base_h), 255, thickness=-1)
            bg_removed_transparent_work = cv2.rectangle(bg_removed_transparent_work, (0, 0), (int(bg_removed_transparent_work.shape[1]), int(bg_removed_transparent_work.shape[0]/2)-work_base_h[0]), bg, thickness=-1)
            bg_removed_transparent_work = cv2.rectangle(bg_removed_transparent_work, (0, int(bg_removed_transparent_work.shape[0]/2)-work_base_h[1]), (bg_removed_transparent_work.shape[1], bg_removed_transparent_work.shape[0]), bg, thickness=-1)

            cv2.imwrite('./images/{}_gray.jpg'.format(self.name), gray)
            cv2.imwrite('./images/{}_depth_map.jpg'.format(self.name), depth_map)
            cv2.imwrite('./images/{}_transparent_work.jpg'.format(self.name), transparent_work)
            cv2.imwrite('./images/{}_bg_removed_transparent_work.jpg'.format(self.name), bg_removed_transparent_work)
            #if belt:
            #    est_range = self.calculate_work_y_mm(belt, self.depth_frame, gray_img, work_base_h, camera_y, grasping_center_y)
            #else: 
            #    est_range = self.calculate_work_y_mm(belt, self.depth_frame, ref_img, work_base_h, camera_y, grasping_center_y)

            #return est_range

            #self.pipeline.stop()
            #cv2.destroyAllWindows()

        except KeyboardInterrupt:
            #self.pipeline.stop()
            cv2.destroyAllWindows()

def main():
    inst = module_object_detector()
    time.sleep(3)
    inst.take_photo()
    inst.pipeline.stop()

if __name__ == "__main__":
    main()
