import pyrealsense2 as rs
import numpy as np
import cv2


class CameraController:

    IMAGE_CENTER_X = 320
    IMAGE_CENTER_Y = 240

    COLOR_RANGES = {

        "red": [
            ((0,100,100),(10,255,255)),
            ((170,100,100),(180,255,255))
        ],

        "blue": [
            ((100,100,50),(130,255,255))
        ],

        "green": [
            ((40,50,50),(90,255,255))
        ],

        "yellow": [
            ((20,100,100),(35,255,255))
        ]
    }

    def __init__(self):

        self.pipeline = rs.pipeline()

        config = rs.config()

        config.enable_stream(
            rs.stream.color,
            640,
            480,
            rs.format.bgr8,
            30
        )

        config.enable_stream(
            rs.stream.depth,
            640,
            480,
            rs.format.z16,
            30
        )

        self.config = config
        self.align = rs.align(rs.stream.color)

    def start(self):
        self.pipeline.start(self.config)

    def stop(self):
        self.pipeline.stop()

    def get_frames(self):

        frames = self.pipeline.wait_for_frames()

        aligned = self.align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame:
            return None, None

        color_image = np.asanyarray(
            color_frame.get_data()
        )

        return color_image, depth_frame

    def detect_target(self, target_color):

        color_image, depth_frame = self.get_frames()

        if color_image is None:
            return None

        hsv = cv2.cvtColor(
            color_image,
            cv2.COLOR_BGR2HSV
        )

        if target_color not in self.COLOR_RANGES:
            return None

        mask = None

        for lower, upper in self.COLOR_RANGES[target_color]:

            temp = cv2.inRange(
                hsv,
                np.array(lower),
                np.array(upper)
            )

            if mask is None:
                mask = temp
            else:
                mask |= temp

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:

            cv2.imshow(
                "Visual Servo",
                color_image
            )

            cv2.waitKey(1)

            return None

        contour = max(
            contours,
            key=cv2.contourArea
        )

        if cv2.contourArea(contour) < 300:
            return None

        M = cv2.moments(contour)

        if M["m00"] == 0:
            return None

        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])

        rect = cv2.minAreaRect(contour)

        angle = rect[2]

        box = cv2.boxPoints(rect)
        box = np.intp(box)

        cv2.drawContours(
            color_image,
            [box],
            0,
            (255,0,255),
            2
        )

        cv2.circle(
            color_image,
            (cx,cy),
            8,
            (0,0,255),
            -1
        )

        cv2.circle(
            color_image,
            (
                self.IMAGE_CENTER_X,
                self.IMAGE_CENTER_Y
            ),
            8,
            (0,255,0),
            -1
        )

        cv2.line(
            color_image,
            (
                self.IMAGE_CENTER_X,
                self.IMAGE_CENTER_Y
            ),
            (cx,cy),
            (255,0,0),
            2
        )

        cv2.imshow(
            "Visual Servo",
            color_image
        )

        cv2.waitKey(1)

        return {
            "cx": cx,
            "cy": cy,
            "angle": angle
        }

    def get_center_depth(self):

        _, depth_frame = self.get_frames()

        return (
            depth_frame.get_distance(
                self.IMAGE_CENTER_X,
                self.IMAGE_CENTER_Y
            ) * 1000
        )