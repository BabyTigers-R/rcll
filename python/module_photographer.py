# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import sys
import usb

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
    serial_numbers = map(lambda device: device.get_info(rs.camera_info.serial_number), devices)

    serial_numbers_ = list(serial_numbers)[:max_n]

    return serial_numbers_

class module_photographer():
    def __init__(self, name, gazebo=False):
        self.gazebo = gazebo
        self.name = name
        self.WIDTH = 640
        self.HEIGHT = 480

        reset_realsense_devices()

        serial_numbers = get_realsense_serialnumbers(max_n=1)
        if len(serial_numbers) == 0:
            print("Not found realsense devices")
            return

        # ストリーミング初期化
        self.config = rs.config()
        #print(serial_numbers[0])
        self.config.enable_device(str(serial_numbers[0]))
        self.config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 15)
        self.config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 15)

        # ストリーミング開始
        self.pipeline = rs.pipeline()
        #self.profile = self.pipeline.start(self.config)
        # Alignオブジェクト生成
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        #self.pipeline.stop()

    def run(self):
        if self.gazebo:
            return 0
        self.profile = self.pipeline.start(self.config)
        time.sleep(3)
        range_min = 0.1
        range_max = 0.20
        bg = 0
        white_color = 255
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        try:
            frames = self.pipeline.wait_for_frames(30000)
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            color_image = cv2.resize(color_image, (self.WIDTH, self.HEIGHT))
            depth_image = cv2.resize(depth_image, (640, 480))

            # self.range_max以内を画像化
            depth = depth_image.astype(np.float64) * depth_scale
            mask = np.where((depth > range_min) & (depth < range_max), white_color, bg)
            mask = mask.astype(np.uint8)
            ref_img = mask

            kernel = np.ones((15,15),np.uint8)
            ref_img = cv2.morphologyEx(ref_img, cv2.MORPH_OPEN, kernel)
            ref_img = cv2.morphologyEx(ref_img, cv2.MORPH_CLOSE, kernel)

            gray_image = cv2.cvtColor(color_image.copy(), cv2.COLOR_BGR2GRAY)
            #bg_removed = (mask/255).astype(np.uint8) * gray_image

            #refference_h = 0
            #cv2.rectangle(ref_img, (0, 0), (int(ref_img.shape[1]), int(ref_img.shape[0]/2)+refference_h), 255, thickness=-1)
            #depth_map = ((depth.astype(np.float64) / (2**64)) * 255).astype(np.uint8)

            cv2.imwrite('./images/{}_gray.jpg'.format(self.name), gray_image)
            cv2.imwrite('./images/{}_map.jpg'.format(self.name), depth_image)
            cv2.imwrite('./images/{}_pre.jpg'.format(self.name), mask)
            cv2.imwrite('./images/{}.jpg'.format(self.name), ref_img)

            # ストリーミング停止
            self.pipeline.stop()
            cv2.destroyAllWindows()

        except KeyboardInterrupt:
            # ストリーミング停止
            self.pipeline.stop()
            cv2.destroyAllWindows()


def main():
    #jpg画像の名前
    if len(sys.argv) > 1:
        name = sys.argv[1]
    else:
        print("Error! Please set args!!!")
        quit()

    inst = module_photographer(name)
    inst.run()

if __name__ == "__main__":
    main()
