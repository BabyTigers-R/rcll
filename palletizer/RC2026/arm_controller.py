# ==========================================================
# arm_controller.py
# Part 3-1
# ==========================================================

import math
import time

import rclpy
import geometry_msgs.msg 
from pymycobot import MyPalletizerSocket

import config
from camera_controller import CameraController


# ==========================================================
# PID
# ==========================================================

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

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):

        self.integral = 0.0
        self.prev_error = 0.0

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
  
        

# ==========================================================
# Arm Controller
# ==========================================================

class ArmController:

    def __init__(self):

        # --------------------------------------
        # Camera
        # --------------------------------------

        self.camera = CameraController()

        # --------------------------------------
        # Robot
        # --------------------------------------

        self.mc = MyPalletizerSocket(

            "192.168.0.232",

            9000

        )

        self.mc.connect_socket()

        # --------------------------------------
        # PID
        # --------------------------------------

        self.pid_x = PIDController(

            config.PID_X_KP,

            config.PID_X_KI,

            config.PID_X_KD

        )

        self.pid_y = PIDController(

            config.PID_Y_KP,

            config.PID_Y_KI,

            config.PID_Y_KD

        )


        # --------------------------------------
        # ROS
        # --------------------------------------


        rclpy.init(args=None)

        self.node = rclpy.create_node("arm_controller")

        stamped = self.node.declare_parameter('stamped', False).value
        if stamped:
            self.TwistMsg = geometry_msgs.msg.TwistStamped
        else:
            self.TwistMsg = geometry_msgs.msg.Twist
        self.cmd_vel_pub = self.node.create_publisher(
            self.TwistMsg,
            "/cmd_vel",
            10
        )

    # ======================================================
    # Safe API
    # ======================================================

    def get_coords_safe(
        self,
        retry=100
    ):

        for i in range(retry):

            coords = self.mc.get_coords()

            print(
                f"get_coords[{i}] =",
                coords
            )

            if isinstance(
                coords,
                (
                    list,
                    tuple
                )
            ):

                return coords

            time.sleep(0.2)

        raise Exception(
            "get_coords failed"
        )

    def get_angles_safe(
        self,
        retry=100
    ):

        for i in range(retry):

            angles = self.mc.get_angles()

            print(
                f"get_angles[{i}] =",
                angles
            )

            if isinstance(
                angles,
                (
                    list,
                    tuple
                )
            ):

                return angles

            time.sleep(0.2)

        raise Exception(
            "get_angles failed"
        )
    
    def coords_range_checker(self,coords):
        print("chek now")
        x2_y2=coords[0]**2 + coords[1]**2
        print(f"x2_y2={x2_y2}")
        r2 = config.ARM_RADIUS**2
        if x2_y2 >r2:
            if coords[0]>coords[1]:
                if coords[0] > 0:
                    print("OUT OF RANGE(X+)")
                    self.shutdown()
                    return "move_x+"
                else:
                    print("OUT OF RANGE(X-)")
                    self.shutdown()
                    return "move_x-"
            else:
                if coords[1] > 0:
                    print("OUT OF RANGE(Y+)")
                    self.shutdown()
                    return "move_y+"
                else:
                    print("OUT OF RANGE(Y-)")
                    self.shutdown()
                    return "move_y-"
        
        if coords[2] > config.Z_COORD_UPPER:
            print("OUT OF RANGE(Z_UPPER)")
            self.shutdown()
            return 
        if coords[2] < config.Z_COORD_LOWER:
            print("OUT OF RANGE(Z_LOWER)")
            self.shutdown()
            return 
    # def coodrs_range_checker(self,coords):
    #     if coords[0] > config.X_COORD_UPPER:
    #         print("OUT OF RANGE(X_UPPER)")
    #         return "move_x+"
    #     if coords[0] < config.X_COORD_LOWER:
    #         print("OUT OF RANGE(X_LOWER)")
    #         return "move_x-"
        
    #     if coords[1] > config.Y_COORD_UPPER:
    #         print("OUT OF RANGE(Y_UPPER)")
    #         return "move_y+"
    #     if coords[1] < config.Y_COORD_LOWER:
    #         print("OUT OF RANGE(Y_LOWER)")
    #         return "move_y-"
        
    #     if coords[2] > config.Z_COORD_UPPER:
    #         print("OUT OF RANGE(Z_UPPER)")
    #         return 
    #     if coords[2] < config.Z_COORD_LOWER:
    #         print("OUT OF RANGE(Z_LOWER)")
    #         return 
        
    # def anlges_range_checker(self,angles):
    #     if angles[0] > config.J1_ANGLE_UPPER:
    #         print("OUT OF RANGE(J1_UPPER)")
    #         return 
    #     if angles[0] < config.J1_ANGLE_LOWER:
    #         print("OUT OF RANGE(J1_LOWER)")
    #         return 
        
    #     if angles[1] > config.J2_ANGLE_UPPER:
    #         print("OUT OF RANGE(j2_UPPER)")
    #         return 
    #     if angles[1] < config.J2_ANGLE_LOWER:
    #         print("OUT OF RANGE(j2_LOWER)")
    #         return 
        
    #     if angles[2] > config.J3_ANGLE_UPPER:
    #         print("OUT OF RANGE(j2_UPPER)")
    #         return 
    #     if angles[2] < config.J3_ANGLE_LOWER:
    #         print("OUT OF RANGE(j3_LOWER)")
    #         return 
        
        
        
    # ======================================================
    # Motion
    # ======================================================

    def move_home(self):

        self.mc.sync_send_angles(

            config.HOME_ANGLES,

            config.SEARCH_SPEED

        )

    def move_search_pose(self):

        self.mc.sync_send_coords(

            config.SEARCH_COORDS,

            config.SEARCH_SPEED,

            15

        )

    # ======================================================
    # Camera
    # ======================================================

    def start_camera(self):

        self.camera.start()

    def stop_camera(self):

        self.camera.stop()

# ==========================================================
# arm_controller.py
# Part 3-2
# ==========================================================

    # ======================================================
    # Visual Servo
    # ======================================================

    def visual_servo(
        self,
        target_color,
        target_shape
    ):

        self.pid_x.reset()
        self.pid_y.reset()

        while True:

            target = self.camera.update(

                target_color,

                target_shape

            )

            if target is None:

                continue

            error_x = (

                target["cx"]

                -

                config.IMAGE_CENTER_X

            )

            error_y = (

                target["cy"]

                -

                config.IMAGE_CENTER_Y

            )

            print("--------------------------------")

            print("Target")

            print(target)

            print()

            print(

                "Error X :",

                error_x

            )

            print(

                "Error Y :",

                error_y

            )

            # --------------------------------------
            # Finish
            # --------------------------------------

            if (

                abs(error_x)

                <=

                config.CENTER_TOLERANCE

                and

                abs(error_y)

                <=

                config.CENTER_TOLERANCE

            ):

                print("Visual Servo Finished")

                return target

            # --------------------------------------
            # PID
            # --------------------------------------

            move_x = self.pid_x.update(

                error_x

            )

            move_y = self.pid_y.update(

                error_y

            )

            coords = self.get_coords_safe()

            # --------------------------------------
            # Camera Coordinate
            #
            # Image
            #
            # +X →
            # +Y ↓
            #
            # Robot
            #
            # +X Forward
            # +Y Left
            #
            # Image Down
            #      ↓
            # Robot +X
            #
            # Image Right
            #      →
            # Robot -Y
            # --------------------------------------
            if coords[2] <= 110:
                coords[2] = 150
            target_coords = [

                coords[0] - move_y,

                coords[1] - move_x,

                coords[2],

                coords[3]

            ]

            self.coords_range_checker(target_coords)
            print()

            print("Before")

            print(coords)

            print()

            print("After")

            print(target_coords)

            self.mc.sync_send_coords(

                target_coords,

                config.MOVE_SPEED,

                15

            )

            time.sleep(0.05)

# ==========================================================
# arm_controller.py
# Part 3-3
# ==========================================================

    # ======================================================
    # Camera -> Gripper Alignment
    # ======================================================

    def align_gripper(self):

        angles = self.get_angles_safe()

        depth = self.camera.get_center_depth()

        # --------------------------------------
        # Camera pitch compensation
        # --------------------------------------

        pitch_offset = (

            depth *

            math.tan(

                config.CAMERA_PITCH_RAD

            )

        )

        camera_x = (

            config.CAMERA_OFFSET_X

            -

            pitch_offset

        )

        camera_y = config.CAMERA_OFFSET_Y

        joint1 = math.radians(

            angles[0]

        )

        offset_x = (

            camera_x *

            math.cos(joint1)

            -

            camera_y *

            math.sin(joint1)

        )

        offset_y = (

            camera_x *

            math.sin(joint1)

            +

            camera_y *

            math.cos(joint1)

        )
        print("depth =", depth)
        print("pitch_offset =", pitch_offset)
        print("camera_x =", camera_x)
        print("camera_y =", camera_y)
        print("offset_x =", offset_x)
        print("offset_y =", offset_y)
        coords = self.get_coords_safe()

        

        
        target = [

            coords[0] + offset_x,

            coords[1] + offset_y,

            coords[2],

            coords[3]

        ]
        self.coords_range_checker(target)
        print()

        print("Camera Alignment")

        print(target)

        self.mc.sync_send_coords(

            target,

            config.SEARCH_SPEED,

            15

        )

    # ======================================================
    # Rotate Gripper
    # ======================================================

    def rotate_gripper(
        self,
        lego_angle
    ):

        angles = self.get_angles_safe()

        angles[3] = lego_angle + config.GRIPPER_ANGLE_OFFSET

        print()

        print("Rotate")

        print(lego_angle)

        self.mc.sync_send_angles(

            angles,

            config.MOVE_SPEED

        )

    # ======================================================
    # Open Gripper
    # ======================================================

    def open_gripper(self):

        self.mc.set_gripper_state(

            config.GRIPPER_OPEN,

            config.GRIPPER_SPEED

        )

        time.sleep(1.5)

    # ======================================================
    # Close Gripper
    # ======================================================

    def close_gripper(self):

        self.mc.set_gripper_value(

            config.GRIPPER_CLOSE,

            config.GRIPPER_SPEED

        )

        time.sleep(2)
# ==========================================================
# arm_controller.py
# Part 3-4 (Final)
# ==========================================================

    # ======================================================
    # Descend
    # ======================================================

    def descend_and_grasp(self):

        if config.USE_DEPTH:

            depth = self.camera.get_center_depth()

        else:

            depth = config.DEFAULT_DEPTH

        descend = (

            depth

            -

            config.LEGO_HEIGHT

            -

            config.TOOL_OFFSET

        )

        coords = self.get_coords_safe()

        target = [

            coords[0],

            coords[1],

            coords[2] - descend,

            coords[3]

        ]

        print()

        print("Descend")

        print(target)

        # 開く
        self.open_gripper()

        self.mc.sync_send_coords(

            target,

            config.MOVE_SPEED,

            15

        )

        time.sleep(1)

        # 掴む
        self.close_gripper()

    # ======================================================
    # Lift
    # ======================================================

    def lift(self):

        coords = self.get_coords_safe()

        target = [

            coords[0],

            coords[1],

            coords[2] + 100,

            coords[3]

        ]
        self.coords_range_checker(target)
        print()

        print("Lift")

        print(target)

        self.mc.sync_send_coords(

            target,

            config.SEARCH_SPEED,

            15

        )

    # ======================================================
    # Pick Sequence
    # ======================================================

    def pick(

        self,

        target_color,

        target_shape

    ):

        print()

        print("==============================")

        print("START PICK")

        print("==============================")

        target = self.visual_servo(

            target_color,

            target_shape

        )

        print()

        print("Align Gripper")
        

        self.align_gripper()

        print()

        print("Rotate Gripper")

        self.rotate_gripper(

            target["angle"]

        )

        print()

        print("Descend")

        self.descend_and_grasp()

        print()

        print("Lift")

        self.lift()

        print()

        print("==============================")

        print("FINISH PICK")

        print("==============================")

        return target

    # ======================================================
    # Shutdown
    # ======================================================

    def shutdown(self):

        try:

            self.camera.stop()

        except:

            pass

        try:
            self.mc.sync_send_coords([150,0,80,0],10,15)
            time.sleep(1)
            # self.mc.release_all_servos()
            self.mc.set_gripper_state(10,100)
            

        except:

            pass

        print("Shutdown Complete")
        
        
    #======================================================
    #Move myAGV
    #======================================================

    def send_cmd_vel(self, vx, vy, wz):

        msg = self.TwistMsg()

        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = wz

        self.cmd_vel_pub.publish(msg)

        rclpy.spin_once(self.node, timeout_sec=0)
