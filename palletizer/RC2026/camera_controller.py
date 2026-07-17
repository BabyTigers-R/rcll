# ==========================================================
# camera_controller.py
# Part 2-1
# ==========================================================

import math
import time

import cv2
import numpy as np
import pyrealsense2 as rs

from ultralytics import YOLO

import config


class CameraController:

    def __init__(self):

        print("Loading YOLO model...")

        self.model = YOLO(
            config.YOLO_MODEL
        )

        print("YOLO loaded.")

        # ---------------------------------------
        # RealSense
        # ---------------------------------------

        self.pipeline = rs.pipeline()

        self.rs_config = rs.config()

        self.rs_config.enable_stream(
            rs.stream.color,
            config.IMAGE_WIDTH,
            config.IMAGE_HEIGHT,
            rs.format.bgr8,
            30
        )

        self.rs_config.enable_stream(
            rs.stream.depth,
            config.IMAGE_WIDTH,
            config.IMAGE_HEIGHT,
            rs.format.z16,
            30
        )

        self.align = rs.align(
            rs.stream.color
        )

        # ---------------------------------------
        # Runtime
        # ---------------------------------------

        self.running = False

        self.color_image = None

        self.depth_frame = None

        # ---------------------------------------
        # FPS
        # ---------------------------------------

        self.prev_time = time.time()

        self.fps = 0

        # ---------------------------------------
        # Tracking
        # ---------------------------------------

        self.roi = None

        self.lost_count = 0

        # ---------------------------------------
        # Debug
        # ---------------------------------------

        self.last_target = None

        self.last_mask = None

    # ==========================================================
    # Camera
    # ==========================================================

    def start(self):

        if self.running:
            return

        self.pipeline.start(
            self.rs_config
        )

        self.running = True

        print("Camera Started")

    def stop(self):

        if not self.running:
            return

        self.pipeline.stop()

        self.running = False

        cv2.destroyAllWindows()

        print("Camera Stopped")

    # ==========================================================
    # FPS
    # ==========================================================

    def update_fps(self):

        now = time.time()

        dt = now - self.prev_time

        if dt > 0:

            self.fps = 1.0 / dt

        self.prev_time = now

    # ==========================================================
    # Frame
    # ==========================================================

    def get_frames(self):

        if not self.running:

            return None, None

        frames = self.pipeline.wait_for_frames()

        aligned = self.align.process(
            frames
        )

        color_frame = aligned.get_color_frame()

        depth_frame = aligned.get_depth_frame()

        if (
            not color_frame
            or
            not depth_frame
        ):

            return None, None

        self.color_image = np.asanyarray(
            color_frame.get_data()
        )

        self.depth_frame = depth_frame

        self.update_fps()

        return (
            self.color_image,
            self.depth_frame
        )

    # ==========================================================
    # Depth
    # ==========================================================

    def get_center_depth(self):

        if not config.USE_DEPTH:

            return config.DEFAULT_DEPTH

        if self.depth_frame is None:

            return config.DEFAULT_DEPTH

        depth = self.depth_frame.get_distance(

            config.IMAGE_CENTER_X,

            config.IMAGE_CENTER_Y

        )

        if depth <= 0:

            return config.DEFAULT_DEPTH

        return depth * 1000

    def get_target_depth(
        self,
        cx,
        cy
    ):

        if not config.USE_DEPTH:

            return config.DEFAULT_DEPTH

        if self.depth_frame is None:

            return config.DEFAULT_DEPTH

        depth = self.depth_frame.get_distance(
            int(cx),
            int(cy)
        )

        if depth <= 0:

            return config.DEFAULT_DEPTH

        return depth * 1000

    # ==========================================================
    # ROI
    # ==========================================================

    def create_roi(
        self,
        cx,
        cy
    ):

        size = config.ROI_SIZE

        half = size // 2

        x1 = max(
            0,
            int(cx - half)
        )

        y1 = max(
            0,
            int(cy - half)
        )

        x2 = min(
            config.IMAGE_WIDTH,
            int(cx + half)
        )

        y2 = min(
            config.IMAGE_HEIGHT,
            int(cy + half)
        )

        self.roi = (
            x1,
            y1,
            x2,
            y2
        )

    def update_roi(
        self,
        cx,
        cy
    ):

        if self.roi is None:

            self.create_roi(
                cx,
                cy
            )

            return

        ox1, oy1, ox2, oy2 = self.roi

        old_cx = (ox1 + ox2) / 2

        old_cy = (oy1 + oy2) / 2

        alpha = config.ROI_SMOOTH_ALPHA

        cx = old_cx * (1 - alpha) + cx * alpha

        cy = old_cy * (1 - alpha) + cy * alpha

        self.create_roi(
            cx,
            cy
        )
        # ==========================================================
# camera_controller.py
# Part 2-2
# ==========================================================

    # ==========================================================
    # YOLO Detection
    # ==========================================================

    def detect_yolo(
        self,
        image,
        target_shape
    ):

        results = self.model.predict(

            source=image,

            conf=config.YOLO_CONFIDENCE,

            iou=config.YOLO_IOU,

            verbose=False

        )

        candidates = []

        if len(results) == 0:

            return candidates

        result = results[0]

        if result.boxes is None:

            return candidates

        for box in result.boxes:

            cls = int(box.cls[0])

            class_name = self.model.names[cls]

            if class_name != target_shape:

                continue

            confidence = float(box.conf[0])

            x1, y1, x2, y2 = (

                box.xyxy[0]

                .cpu()

                .numpy()

                .astype(int)

            )

            area = (x2 - x1) * (y2 - y1)

            candidates.append(

                {

                    "bbox": (

                        x1,

                        y1,

                        x2,

                        y2

                    ),

                    "shape": class_name,

                    "confidence": confidence,

                    "area": area

                }

            )

        candidates.sort(

            key=lambda x: x["area"],

            reverse=True

        )

        return candidates


    # ==========================================================
    # HSV Color Detection
    # ==========================================================

    def detect_color(
        self,
        roi
    ):

        hsv = cv2.cvtColor(

            roi,

            cv2.COLOR_BGR2HSV

        )

        mask_red = (

            cv2.inRange(

                hsv,

                np.array(config.HSV_RED_1[0]),

                np.array(config.HSV_RED_1[1])

            )

            |

            cv2.inRange(

                hsv,

                np.array(config.HSV_RED_2[0]),

                np.array(config.HSV_RED_2[1])

            )

        )

        mask_blue = cv2.inRange(

            hsv,

            np.array(config.HSV_BLUE[0]),

            np.array(config.HSV_BLUE[1])

        )

        mask_yellow = cv2.inRange(

            hsv,

            np.array(config.HSV_YELLOW[0]),

            np.array(config.HSV_YELLOW[1])

        )

        mask_green = cv2.inRange(

            hsv,

            np.array(config.HSV_GREEN[0]),

            np.array(config.HSV_GREEN[1])

        )

        kernel = np.ones(

            (

                config.KERNEL_SIZE,

                config.KERNEL_SIZE

            ),

            np.uint8

        )

        mask_red = cv2.morphologyEx(

            mask_red,

            cv2.MORPH_OPEN,

            kernel

        )

        mask_red = cv2.morphologyEx(

            mask_red,

            cv2.MORPH_CLOSE,

            kernel

        )

        mask_blue = cv2.morphologyEx(

            mask_blue,

            cv2.MORPH_OPEN,

            kernel

        )

        mask_blue = cv2.morphologyEx(

            mask_blue,

            cv2.MORPH_CLOSE,

            kernel

        )

        mask_yellow = cv2.morphologyEx(

            mask_yellow,

            cv2.MORPH_OPEN,

            kernel

        )

        mask_yellow = cv2.morphologyEx(

            mask_yellow,

            cv2.MORPH_CLOSE,

            kernel

        )

        mask_green = cv2.morphologyEx(

            mask_green,

            cv2.MORPH_OPEN,

            kernel

        )

        mask_green = cv2.morphologyEx(

            mask_green,

            cv2.MORPH_CLOSE,

            kernel

        )

        masks = {

            config.COLOR_RED: mask_red,

            config.COLOR_BLUE: mask_blue,

            config.COLOR_YELLOW: mask_yellow,

            config.COLOR_GREEN: mask_green

        }

        areas = {}

        for color, mask in masks.items():

            areas[color] = cv2.countNonZero(mask)

        detected_color = max(

            areas,

            key=areas.get

        )

        self.last_mask = masks[detected_color]

        return (

            detected_color,

            masks[detected_color]

        )
# ==========================================================
# camera_controller.py
# Part 2-3
# ==========================================================

    # ==========================================================
    # Contour Detection
    # ==========================================================

    def detect_contour(
        self,
        mask,
        offset_x=0,
        offset_y=0
    ):

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            return None

        contour = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(contour)

        if area < config.MIN_CONTOUR_AREA:
            return None

        M = cv2.moments(contour)

        if M["m00"] == 0:
            return None

        cx = int(
            M["m10"] /
            M["m00"]
        )

        cy = int(
            M["m01"] /
            M["m00"]
        )

        rect = cv2.minAreaRect(
            contour
        )

        (
            center,
            size,
            angle
        ) = rect

        w = size[0]
        h = size[1]

        # ----------------------------------
        # 長辺方向へ補正
        # ----------------------------------

        if w < h:
            angle += 90

        if angle > 90:
            angle -= 180

        depth = self.get_target_depth(
            cx + offset_x,
            cy + offset_y
        )

        return {

            "cx":
                cx + offset_x,

            "cy":
                cy + offset_y,

            "angle":
                angle,

            "depth":
                depth,

            "area":
                area,

            "contour":
                contour
        }


    # ==========================================================
    # Target Detection
    # ==========================================================

    def detect_target(
        self,
        target_color,
        target_shape
    ):

        color_image, depth_frame = self.get_frames()

        if color_image is None:
            return None

        # ======================================================
        # SEARCH MODE
        # ======================================================

        if self.roi is None:

            candidates = self.detect_yolo(
                color_image,
                target_shape
            )

            if len(candidates) == 0:

                self.last_target = None

                return None

            h, w = color_image.shape[:2]

            for candidate in candidates:

                x1, y1, x2, y2 = candidate["bbox"]

                x1 = max(
                    0,
                    x1 - config.ROI_MARGIN
                )

                y1 = max(
                    0,
                    y1 - config.ROI_MARGIN
                )

                x2 = min(
                    w,
                    x2 + config.ROI_MARGIN
                )

                y2 = min(
                    h,
                    y2 + config.ROI_MARGIN
                )

                roi = color_image[
                    y1:y2,
                    x1:x2
                ]

                if roi.size == 0:
                    continue

                color, mask = self.detect_color(
                    roi
                )

                if color != target_color:
                    continue

                target = self.detect_contour(

                    mask,

                    x1,

                    y1

                )

                if target is None:
                    continue

                target["shape"] = target_shape
                target["color"] = target_color
                target["bbox"] = (
                    x1,
                    y1,
                    x2,
                    y2
                )

                self.create_roi(

                    target["cx"],

                    target["cy"]

                )

                self.lost_count = 0

                self.last_target = target

                return target

            self.last_target = None

            return None

        # ======================================================
        # TRACK MODE
        # ======================================================

        x1, y1, x2, y2 = self.roi

        x1 = max(0, x1)
        y1 = max(0, y1)

        x2 = min(
            config.IMAGE_WIDTH,
            x2
        )

        y2 = min(
            config.IMAGE_HEIGHT,
            y2
        )

        roi = color_image[
            y1:y2,
            x1:x2
        ]

        if roi.size == 0:

            self.roi = None

            self.last_target = None

            return None

        color, mask = self.detect_color(
            roi
        )

        if color != target_color:

            self.lost_count += 1

            if self.lost_count >= config.ROI_LOST_LIMIT:

                self.roi = None

                self.lost_count = 0

            self.last_target = None

            return None

        target = self.detect_contour(

            mask,

            x1,

            y1

        )

        if target is None:

            self.lost_count += 1

            if self.lost_count >= config.ROI_LOST_LIMIT:

                self.roi = None

                self.lost_count = 0

            self.last_target = None

            return None

        self.lost_count = 0

        self.update_roi(

            target["cx"],

            target["cy"]

        )

        target["shape"] = target_shape
        target["color"] = target_color
        target["bbox"] = self.roi

        self.last_target = target

        return target
    
# ==========================================================
# camera_controller.py
# Part 2-4 (Final)
# ==========================================================

    # ==========================================================
    # Debug Draw
    # ==========================================================

    def draw_debug(self):

        if not config.SHOW_WINDOW:
            return

        if self.color_image is None:
            return

        image = self.color_image.copy()

        # ------------------------------------------------------
        # Image Center
        # ------------------------------------------------------

        if config.SHOW_CENTER:

            cv2.drawMarker(

                image,

                (
                    config.IMAGE_CENTER_X,
                    config.IMAGE_CENTER_Y
                ),

                config.COLOR_CENTER_POINT,

                cv2.MARKER_CROSS,

                20,

                2

            )

        # ------------------------------------------------------
        # ROI
        # ------------------------------------------------------

        if (

            config.SHOW_ROI

            and

            self.roi is not None

        ):

            x1, y1, x2, y2 = self.roi

            cv2.rectangle(

                image,

                (x1, y1),

                (x2, y2),

                config.COLOR_ROI,

                2

            )

        # ------------------------------------------------------
        # Target
        # ------------------------------------------------------

        if self.last_target is not None:

            target = self.last_target

            # ----------------------------
            # Bounding Box
            # ----------------------------

            if config.SHOW_YOLO:

                x1, y1, x2, y2 = target["bbox"]

                cv2.rectangle(

                    image,

                    (x1, y1),

                    (x2, y2),

                    config.COLOR_BOX,

                    2

                )

            # ----------------------------
            # Contour
            # ----------------------------

            if config.SHOW_CONTOUR:

                contour = target["contour"].copy()

                contour[:, :, 0] -= target["bbox"][0]

                contour[:, :, 1] -= target["bbox"][1]

                cv2.drawContours(

                    image[
                        target["bbox"][1]:
                        target["bbox"][3],

                        target["bbox"][0]:
                        target["bbox"][2]
                    ],

                    [contour],

                    -1,

                    config.COLOR_CONTOUR,

                    2

                )

            # ----------------------------
            # Center
            # ----------------------------

            if config.SHOW_TARGET:

                cv2.circle(

                    image,

                    (
                        int(target["cx"]),
                        int(target["cy"])
                    ),

                    5,

                    config.COLOR_CENTER_POINT,

                    -1

                )

            # ----------------------------
            # Angle
            # ----------------------------

            if config.SHOW_ANGLE:

                length = 70

                rad = math.radians(

                    target["angle"]

                )

                x2 = int(

                    target["cx"]

                    +

                    length * math.cos(rad)

                )

                y2 = int(

                    target["cy"]

                    +

                    length * math.sin(rad)

                )

                cv2.line(

                    image,

                    (

                        int(target["cx"]),

                        int(target["cy"])

                    ),

                    (

                        x2,

                        y2

                    ),

                    config.COLOR_DIRECTION,

                    2

                )

            # ----------------------------
            # Text
            # ----------------------------

            if config.SHOW_TEXT:

                cv2.putText(

                    image,

                    f"{target['shape']}",

                    (10, 30),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.7,

                    config.COLOR_TEXT,

                    2

                )

                cv2.putText(

                    image,

                    f"{target['color']}",

                    (10, 60),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.7,

                    config.COLOR_TEXT,

                    2

                )

                cv2.putText(

                    image,

                    f"Angle : {target['angle']:.1f}",

                    (10, 90),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.7,

                    config.COLOR_TEXT,

                    2

                )

                cv2.putText(

                    image,

                    f"Depth : {target['depth']:.1f}",

                    (10, 120),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.7,

                    config.COLOR_TEXT,

                    2

                )

        # ------------------------------------------------------
        # FPS
        # ------------------------------------------------------

        if config.SHOW_FPS:

            cv2.putText(

                image,

                f"FPS : {self.fps:.1f}",

                (10, config.IMAGE_HEIGHT - 20),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.7,

                (0, 255, 0),

                2

            )

        cv2.imshow(

            "Camera",

            image

        )

        cv2.waitKey(1)

    # ==========================================================
    # Update
    # ==========================================================

    def update(

        self,

        target_color,

        target_shape

    ):

        target = self.detect_target(

            target_color,

            target_shape

        )

        self.draw_debug()

        return target