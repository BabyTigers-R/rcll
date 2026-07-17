#!/bin/bash
# Run this inside the ros2_galactic container before launching Nav2, to confirm
# the physical myAGV (myagv_odometry + ydlidar, started on-robot via
# startup_myAGV.sh) is actually visible over ROS2/DDS from this machine.
set -u

source /opt/ros/galactic/setup.bash

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
echo "(myAGV実機側は startup_myAGV.sh で ROS_DOMAIN_ID=0, RMW_IMPLEMENTATION=rmw_fastrtps_cpp に固定されています。上の値が一致しているか確認してください)"
echo

status=0
for topic in /odom /scan /tf; do
  echo -n "checking ${topic} ... "
  if timeout 5 ros2 topic echo --once "${topic}" > /dev/null 2>&1; then
    echo "OK"
  else
    echo "NOT RECEIVED"
    status=1
  fi
done

if [ "${status}" -ne 0 ]; then
  echo
  echo "一部のトピックが受信できませんでした。以下を確認してください:"
  echo "  - myAGV実機側で startup_myAGV.sh (myagv_odometry / ydlidar) が起動しているか"
  echo "  - 実機とこのPCが同じLAN/Wi-Fiに接続されているか"
  echo "  - ROS_DOMAIN_ID / RMW_IMPLEMENTATION が実機とこのコンテナで一致しているか"
fi

exit "${status}"
