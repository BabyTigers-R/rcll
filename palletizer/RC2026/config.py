# ==========================================
# config.py
# RC2026 Vision System Configuration
# ==========================================

import math

# ==========================================
# YOLO
# ==========================================

YOLO_MODEL = "best.pt"

YOLO_CONFIDENCE = 0.60

YOLO_IOU = 0.45


# ==========================================
# Image
# ==========================================

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

IMAGE_CENTER_X = IMAGE_WIDTH // 2
IMAGE_CENTER_Y = IMAGE_HEIGHT // 2


# ==========================================
# Target Classes
# ==========================================

CLASS_2X2 = "LEGO_2_2"
CLASS_2X4 = "LEGO_2_4"


# ==========================================
# Target Colors
# ==========================================

COLOR_RED = "red"
COLOR_BLUE = "blue"
COLOR_YELLOW = "yellow"
COLOR_GREEN = "green"


# ==========================================
# HSV
# ==========================================

HSV_RED_1 = (
    (0, 80, 50),
    (10, 255, 255)
)

HSV_RED_2 = (
    (170, 80, 50),
    (180, 255, 255)
)

HSV_BLUE = (
    (95, 80, 50),
    (135, 255, 255)
)

HSV_YELLOW = (
    (20, 80, 80),
    (40, 255, 255)
)

HSV_GREEN = (
    (40, 60, 40),
    (90, 255, 255)
)


# ==========================================
# Morphology
# ==========================================

KERNEL_SIZE = 5

MIN_CONTOUR_AREA = 200


# ==========================================
# ROI Tracking
# ==========================================

ROI_SIZE = 300

ROI_MARGIN = 30

ROI_SMOOTH_ALPHA = 0.30

ROI_LOST_LIMIT = 5


# ==========================================
# Visual Servo
# ==========================================

CENTER_TOLERANCE = 5

PID_X_KP = 0.30
PID_X_KI = 0.02
PID_X_KD = 0.00

PID_Y_KP = 0.30
PID_Y_KI = 0.02
PID_Y_KD = 0.00


# ==========================================
# Camera Offset
#
# +X : Front
# +Y : Left
# +Z : Up
# ==========================================

CAMERA_OFFSET_X = 75.0
CAMERA_OFFSET_Y = 38.0
CAMERA_OFFSET_Z = 40.0

CAMERA_PITCH_DEG = 8.0

CAMERA_PITCH_RAD = math.radians(
    CAMERA_PITCH_DEG
)


# ==========================================
# Robot
# ==========================================

HOME_ANGLES = [
    0,
    0,
    0,
    0
]

SEARCH_COORDS = [
    150,
    0,
    200,
    0
]

MOVE_SPEED = 20

SEARCH_SPEED = 40


# ==========================================
# Gripper
# ==========================================

GRIPPER_ANGLE_OFFSET = -70

GRIPPER_OPEN = 0

GRIPPER_CLOSE = 20

GRIPPER_SPEED = 50


# ==========================================
# Height
# ==========================================

LEGO_HEIGHT = 10

TOOL_OFFSET = 160


# ==========================================
# Depth
# ==========================================

# Falseにすると固定距離を使用
USE_DEPTH = True

DEFAULT_DEPTH = 210.0


# ==========================================
# Debug
# ==========================================

SHOW_WINDOW = False

SHOW_FPS = True

SHOW_CENTER = True

SHOW_ROI = True

SHOW_YOLO = True

SHOW_CONTOUR = True

SHOW_TARGET = True

SHOW_ANGLE = True

SHOW_TEXT = True


# ==========================================
# Drawing Colors (BGR)
# ==========================================

COLOR_BOX = (255, 0, 0)

COLOR_ROI = (255, 255, 0)

COLOR_CONTOUR = (0, 255, 255)

COLOR_CENTER_POINT = (0, 0, 255)

COLOR_TEXT = (0, 255, 0)

COLOR_DIRECTION = (255, 0, 255)

# ==========================================
# Arm Range
# ==========================================
X_COORD_UPPER = 260
X_COORD_LOWER = -260

Y_COORD_UPPER = 260
Y_COORD_LOWER = -260

Z_COORD_UPPER = 357.58
Z_COORD_LOWER = -15

RX_COORD_UPPER = 180
RX_COORD_LOWER = -180


J1_ANGLE_UPPER = 162
J1_ANGLE_LOWER = -162

J2_ANGLE_UPPER = 90
J2_ANGLE_LOWER = -2

J3_ANGLE_UPPER = 60
J3_ANGLE_LOWER = -92

J4_ANGLE_UPPER = 180
J4_ANGLE_LOWER = -180

ARM_RADIUS = 250