#!/bin/bash
#export ROS_DOMAIN_ID=$(hostname -I | awk '{print $1}' | awk -F. '{print $4}')
#export PS1="(domain:$ROS_DOMAIN_ID) $PS1"
#echo "ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
unset ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

# ros2shell
bash() { :; }
source /home/er/.local/bin/ros2shell
unset -f bash

# startup for LiDAR
# initialization for IO ports
# enable GPIO 20
echo 20 | sudo tee /sys/class/gpio/export
# set output mode
echo out | sudo tee /sys/class/gpio/gpio20/direction
# power on
echo 1 | sudo tee /sys/class/gpio/gpio20/value

source ~/myagv_ros2/install/setup.bash
ros2 launch myagv_odometry myagv_active.launch.py

