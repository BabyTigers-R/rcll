"""ros2 run myagv_navigation go_to_location <location_name>

Sends a Nav2 NavigateToPose goal for a named location (see locations.py)
using nav2_simple_commander, and logs progress until the goal completes.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from myagv_navigation.locations import get_location, list_location_names

FEEDBACK_LOG_EVERY = 10  # ~1s, since isTaskComplete() polls at 10 Hz


def _print_available_locations():
    print('\nAvailable locations\n')
    for name in list_location_names():
        print(name)


def _make_goal_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Usage: ros2 run myagv_navigation go_to_location <location_name>')
        _print_available_locations()
        rclpy.shutdown()
        return 1

    location_name = sys.argv[1]

    try:
        x, y, yaw = get_location(location_name)
    except KeyError:
        print(f"Unknown location '{location_name}'.")
        _print_available_locations()
        rclpy.shutdown()
        return 1

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    goal_pose = _make_goal_pose(navigator, x, y, yaw)

    navigator.info(
        f"Sending goal... -> '{location_name}' "
        f'(x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg)')
    navigator.goToPose(goal_pose)

    loop_count = 0
    while not navigator.isTaskComplete():
        loop_count += 1
        feedback = navigator.getFeedback()
        if feedback and loop_count % FEEDBACK_LOG_EVERY == 0:
            navigator.info(
                f'Distance remaining: {feedback.distance_remaining:.2f} m')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.info(f"Goal reached: '{location_name}'")
        return_code = 0
    elif result == TaskResult.CANCELED:
        navigator.warn(f"Goal to '{location_name}' was canceled!")
        return_code = 1
    else:
        navigator.error(f"Goal to '{location_name}' failed!")
        return_code = 1

    navigator.destroy_node()
    rclpy.shutdown()
    return return_code


if __name__ == '__main__':
    sys.exit(main())
