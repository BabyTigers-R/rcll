#!/bin/bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/kachaka/manual_control/cmd_vel
