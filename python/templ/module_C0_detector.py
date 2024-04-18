# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import sys
import usb

from module_center_of_gravity_detect import module_center_of_gravity_detect

ID_VENDOR_REALSENSE = 0x8086 # Intel
MANUFACTURER_REALSENSE = "Intel(R) RealSense(TM)"
PRODUCT_REALSENSE = "Intel(R) RealSense(TM)"

def reset_realsense_devices():
    usb_devices = usb.core.find(find_all=True)

    def is_realsense_device(dev):
        is_same_idVendor = dev.idVendor == ID_VENDOR_REALSENSE
        if not is_same_idVendor:
            return False

        is_same_manufacturer = MANUFACTURER_REALSENSE in dev.manufacturer
        is_same_product = PRODUCT_REALSENSE in dev.product

        return is_same_manufacturer and is_same_product

    realsense_devices = filter(is_realsense_device, usb_devices)

    for dev in realsense_devices:
        dev.reset()

def get_realsense_serialnumbers(max_n=1):
    ctx = rs.context()

    devices = ctx.query_devices()
    for dev in devices:
        print(dev)
    serial_numbers = map(lambda device: device.get_info(rs.camera_info.serial_number), devices)

    serial_numbers_ = list(serial_numbers)[:max_n]

    return serial_numbers_

class module_C0_detector():
    def __init__(self, name, gazebo=False):
        self.gazebo = gazebo
        self.serial_number = "831612071276"
        self.name = name
        self.WIDTH = 640
        self.HEIGHT = 480

        reset_realsense_devices()
        serial_numbers =get_realsense_serialnumbers()

        # RealSense setting
        self.config = rs.config()
        self.config.enable_device(str(serial_numbers[0]))
        # self.config.enable_device(self.serial_number)
        self.config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 15)
        self.config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 15)

        self.pipeline = rs.pipeline()
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.profile = self.pipeline.start(self.config)

    def __call__(self):
        if self.gazebo:
            return 0
        #time.sleep(3)
        range_min = 0.1
        range_max = 0.23
        bg = 0
        white_color = 255
        work_base_h = self.HEIGHT/2
        camera_y = -85
        grasping_center_y = -90
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

            # make pixel which has more than range_max back ground
            depth = depth_image.astype(np.float64) * depth_scale
            mask = np.where((depth > range_min) & (depth < range_max), white_color, bg)
            mask = mask.astype(np.uint8)
            ref_img = mask

            kernel = np.ones((15,15),np.uint8)
            ref_img = cv2.morphologyEx(ref_img, cv2.MORPH_OPEN, kernel)
            ref_img = cv2.morphologyEx(ref_img, cv2.MORPH_CLOSE, kernel)

            gray_image = cv2.cvtColor(color_image.copy(), cv2.COLOR_BGR2GRAY)
            #bg_removed = (mask/255).astype(np.uint8) * gray_image

            #cv2.rectangle(ref_img, (0, 0), (int(ref_img.shape[1]), int(ref_img.shape[0]/2)+refference_h), 255, thickness=-1)
            #depth_map = ((depth.astype(np.float64) / (2**64)) * 255).astype(np.uint8)

            cv2.imwrite('./images/{}_gray.jpg'.format(self.name), gray_image)
            cv2.imwrite('./images/{}_depth.jpg'.format(self.name), depth_image)
            cv2.imwrite('./images/{}_before_noise_removed.jpg'.format(self.name), mask)
            cv2.imwrite('./images/{}_bg_removed.jpg'.format(self.name), ref_img)
            self.calculate_work_y_mm(self.depth_frame, ref_img, work_base_h, camera_y, grasping_center_y)

            #self.pipeline.stop()
            cv2.destroyAllWindows()

        except KeyboardInterrupt:
            #self.pipeline.stop()
            cv2.destroyAllWindows()

    def calculate_work_y_mm(self, depth_frame, ref_img, work_base_h, camera_y, grasping_center_y):
        work_center_pixels = self.calculate_work_y_pixel(ref_img, work_base_h, camera_y, grasping_center_y)
        if work_center_pixels == None:
            return 1000
        
        color_intr = rs.video_stream_profile(self.profile.get_stream(rs.stream.color)).get_intrinsics()

        print(work_center_pixels)
        d_work_center = depth_frame.get_distance(depth_frame, int(work_center_pixels[0][0]), int(work_center_pixels[0][1]))
        point_work_center = rs.rs2_deproject_pixel_to_point(color_intr , work_center_pixels[0], d_work_center)

        d_graspable_center = d_work_center
        point_graspable_center = rs.rs2_deproject_pixel_to_point(color_intr , (int(self.WIDTH), int(self.HEIGHT/2)), d_graspable_center)

        est_range = math.sqrt((point_graspable_center[0]-point_work_center[0])*(point_graspable_center[0]-point_work_center[0]) + (point_graspable_center[1]-point_work_center[1])*(point_graspable_center[1]-point_work_center[1]) +(point_graspable_center[2]-point_work_center[2])*(point_graspable_center[2]-point_work_center[2]))
        print(est_range)

    def calculate_work_y_pixel(self, ref_img, work_base_h, camera_y, grasping_center_y):
        target = ref_img
        cv2.rectangle(target, (0, 0), (self.WIDTH, int(work_base_h-10)), 0, thickness=-1)
        cv2.rectangle(target, (0, int(work_base_h+10)), (self.WIDTH, self.HEIGHT), 0, thickness=-1)
        cv2.imshow("target", target)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        CGD = module_center_of_gravity_detect()
        results = CGD(target)
        CGD.show_result()

        return results

def main():
    if len(sys.argv) > 1:
        name = sys.argv[1]
    else:
        print("Error! Please set args!!!")
        quit()

    inst = module_C0_detector(name)
    time.sleep(3)
    inst()
    inst.pipeline.stop()

if __name__ == "__main__":
    main()
