from arm_controller import ArmController

import time


TARGET_COLOR = "blue"


def main():

    arm = ArmController()

    arm.camera.start()

    print("HOME")
    arm.move_home()

    print("SEARCH")
    arm.move_search_pose()

    time.sleep(2)

    print("VISUAL SERVO")

    target = arm.visual_servo(
        TARGET_COLOR
    )

    print("ALIGN")

    arm.align_gripper()

    print("ROTATE")

    arm.rotate_gripper(
        target["angle"]
    )

    print("GRASP")

    arm.descend_and_grasp()

    print("LIFT")

    arm.lift()

    print("HOME")

    arm.move_home()

    arm.camera.stop()


if __name__ == "__main__":
    main()