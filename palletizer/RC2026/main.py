# ==========================================================
# main.py
# ==========================================================

from arm_controller import ArmController


# ==========================================================
# Select Target
# ==========================================================

# Shape
#
# "LEGO_2_2"
# "LEGO_2_4"
#
TARGET_SHAPE = "LEGO_2_2"


# Color
#
# "red"
# "blue"
# "yellow"
# "green"
#
TARGET_COLOR = "red"


# ==========================================================
# Main
# ==========================================================

def main():

    arm = ArmController()

    try:

        print()

        print("==============================")
        print("RC2026 LEGO PICK SYSTEM")
        print("==============================")

        print()

        print("Starting Camera...")

        arm.start_camera()

        print()

        print("Move Home")

        arm.move_home()

        print()

        print("Move Search Pose")

        arm.move_search_pose()

        print()

        print("------------------------------")
        print("Searching")
        print("------------------------------")

        target = arm.pick(

            TARGET_COLOR,

            TARGET_SHAPE

        )

        print()

        print("------------------------------")
        print("Result")
        print("------------------------------")

        print(target)

        print()

        print("Return Home")

        arm.move_home()

        print()

        print("Mission Complete")

    except KeyboardInterrupt:

        print()

        print("Interrupted")

    except Exception as e:

        print()

        print("Error")

        print(e)

    finally:

        arm.shutdown()


# ==========================================================
# Entry Point
# ==========================================================

if __name__ == "__main__":

    main()