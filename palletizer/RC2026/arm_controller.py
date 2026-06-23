import math
import time

from pymycobot import MyPalletizerSocket

from camera_controller import CameraController


class PIDController:

    def __init__(
        self,
        kp,
        ki,
        kd
    ):

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_error = 0
        self.integral = 0

    def update(
        self,
        error
    ):

        self.integral += error
        derivative = (
            error -
            self.prev_error
        )

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error

        return output


class ArmController:

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

    CAMERA_TO_GRIPPER_DISTANCE = 70

    CENTER_TOLERANCE = 0
    CAMERA_OFFSET_X = 5
    CAMERA_OFFSET_Y = 0
    CAMERA_OFFSET_Z = 0
    camera_angle_offset_phi =math.radians(10)


    LEGO_HEIGHT = 10

    TOOL_OFFSET = 160

    def __init__(self):

        self.camera = CameraController()

        self.mc = MyPalletizerSocket(
            "10.42.10.104",
            9000
        )

        self.mc.connect_socket()

        self.pid_x = PIDController(
            0.3,
            0.0,
            0.0
        )

        self.pid_y = PIDController(
            0.3,
            0.0,
            0.0
        )

    def get_coords_safe(self, retry=100):

        for i in range(retry):

            coords = self.mc.get_coords()

            print(f"get_coords[{i}] =", coords)

            if isinstance(coords, (list, tuple)):
                return coords

            time.sleep(0.2)

        raise Exception("get_coords failed")
        
    def get_angles_safe(self, retry=100):

        for i in range(retry):

            angles = self.mc.get_angles()

            print(f"get_angles[{i}] =", angles)

            if isinstance(angles, (list, tuple)):
                return angles

            time.sleep(0.2)

        raise Exception("get_angles failed")

    def move_home(self):

        self.mc.sync_send_angles(
            self.HOME_ANGLES,
            30
        )

    def move_search_pose(self):

        self.mc.sync_send_coords(
            self.SEARCH_COORDS,
            40,
            15
        )

    def visual_servo(
        self,
        color
    ):

        while True:

            target = (
                self.camera
                .detect_target(color)
            )

            if target is None:
                continue

            error_x = (
                target["cx"]
                -
                self.camera.IMAGE_CENTER_X
            )

            error_y = (
                target["cy"]
                -
                self.camera.IMAGE_CENTER_Y
            )

            if (
                abs(error_x)
                < self.CENTER_TOLERANCE
                and
                abs(error_y)
                < self.CENTER_TOLERANCE
            ):
                return target

            move_x = self.pid_x.update(
                error_x
            )

            move_y = self.pid_y.update(
                error_y
            )

            coords = self.get_coords_safe()

            target_coords = [

                coords[0] - move_y,

                coords[1] - move_x,

                200,

                coords[3]
            ]
            print("error_x =", error_x)
            print("error_y =", error_y)

            print("move_x =", move_x)
            print("move_y =", move_y)

            print("before =", coords)
            print("after  =", target_coords)
            self.mc.sync_send_coords(
                target_coords,
                20,
                15
            )

            time.sleep(0.2)

    def align_gripper(self):

        angles = self.get_angles_safe()
        depth = (
                    self.camera
                    .get_center_depth()
                )
        
        tilt_offset = (
            depth *
            math.tan(self.camera_angle_offset_phi)
        )

        Camera_offset_x = (
            self.CAMERA_OFFSET_X
            + tilt_offset
        )
        joint1 = angles[0]
        
        theta = math.radians(joint1)

        offset_x = (
            Camera_offset_x * math.cos(theta)
            - self.CAMERA_OFFSET_Y * math.sin(theta)
        )

        offset_y = (
            Camera_offset_x * math.sin(theta)
            + self.CAMERA_OFFSET_Y * math.cos(theta)
)

        coords = self.get_coords_safe()

        target = [

            coords[0] + offset_x,

            coords[1] + offset_y,

            coords[2],

            coords[3]
        ]

        self.mc.sync_send_coords(
            target,
            30,
            15
        )

    def rotate_gripper(
        self,
        lego_angle
    ):

        angles = self.get_angles_safe()

        angles[3] = lego_angle

        self.mc.sync_send_angles(
            angles,
            20
        )

    def descend_and_grasp(self):

        depth = (
            self.camera
            .get_center_depth()
        )
        print(depth)
        descend = (
            depth
            -
            self.LEGO_HEIGHT
            -
            self.TOOL_OFFSET
        )

        coords = self.get_coords_safe()

        target = [

            coords[0],

            coords[1],

            coords[2] - descend,

            coords[3]
        ]
        self.mc.set_gripper_state(0,60)
        time.sleep(2)

        self.mc.sync_send_coords(
            target,
            20,
            15
        )

        self.mc.set_gripper_value(
            20,
            50
        )

        time.sleep(2)

    def lift(self):

        coords = self.get_coords_safe()

        target = [

            coords[0],

            coords[1],

            coords[2] + 100,

            coords[3]
        ]

        self.mc.sync_send_coords(
            target,
            30,
            15
        )