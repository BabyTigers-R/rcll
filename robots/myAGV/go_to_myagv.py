#!/usr/bin/env python3
"""Standalone myAGV navigation script — no colcon build / ros2 run required.

Brings up Nav2 (map_server/AMCL/planner/controller/bt_navigator/lifecycle
managers) directly via nav2_bringup's bringup_launch.py — pointed at this
directory's maps/myagv_mapA-v2.yaml and params/nav2_params.yaml — then
sends a goal to a named location and shuts Nav2 down again on exit.

Usage (in a shell with ROS2 sourced, e.g. `source /opt/ros/galactic/setup.bash`):
    python3 go_to_myagv.py <location_name>
    python3 go_to_myagv.py goal
"""
import math
import os
import signal
import subprocess
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_YAML = os.path.join(HERE, 'maps', 'myagv_mapA-v2.yaml')
PARAMS_FILE = os.path.join(HERE, 'params', 'nav2_params.yaml')

FEEDBACK_LOG_EVERY = 10  # ~1s, since isTaskComplete() polls at 10 Hz

# name -> (x [m], y [m], yaw [deg]) in the `map` frame.
LOCATIONS = {
    'start': (0.0, 0.0, 0.0),
    'goal': (2.0, 1.0, 90.0),
    'mps_1': (1.0, 0.5, 0.0),
    'mps_2': (1.0, -0.5, 0.0),
}


def list_location_names():
    return sorted(LOCATIONS.keys())


def get_location(name):
    x, y, yaw_deg = LOCATIONS[name]
    return x, y, math.radians(yaw_deg)


def print_available_locations():
    print('\nAvailable locations\n')
    for name in list_location_names():
        print(name)


def start_nav2():
    """Start Nav2 (map_server/AMCL/planner/controller/bt_navigator) as a
    child process via nav2_bringup's own launch file — no dependency on
    the myagv_navigation ROS2 package or a colcon build."""
    if not os.path.isfile(MAP_YAML):
        raise FileNotFoundError(f'Map file not found: {MAP_YAML}')
    if not os.path.isfile(PARAMS_FILE):
        raise FileNotFoundError(f'Params file not found: {PARAMS_FILE}')

    print(f'Starting Nav2 with map={MAP_YAML}')
    return subprocess.Popen([
        'ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
        f'map:={MAP_YAML}',
        f'params_file:={PARAMS_FILE}',
        'use_sim_time:=false',
    ])


def stop_nav2(process):
    if process.poll() is None:
        print('Shutting down Nav2...')
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()


def make_goal_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 go_to_myagv.py <location_name>')
        print_available_locations()
        return 1

    location_name = sys.argv[1]

    try:
        x, y, yaw = get_location(location_name)
    except KeyError:
        print(f"Unknown location '{location_name}'.")
        print_available_locations()
        return 1

    nav2_process = start_nav2()

    try:
        rclpy.init()
        navigator = BasicNavigator()
        navigator.waitUntilNav2Active()

        goal_pose = make_goal_pose(navigator, x, y, yaw)
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
    finally:
        stop_nav2(nav2_process)

    return return_code


if __name__ == '__main__':
    sys.exit(main())
