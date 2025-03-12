#!/bin/bash
docker run -v ~/ros2_ws:/home/ubuntu/colcon_ws:cached -p 6080:80 -p 10000:10000 --shm-size=1024m tiryoh/ros2-desktop-vnc:humble-arm64
