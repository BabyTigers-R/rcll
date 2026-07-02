FROM osrf/ros:galactic-desktop

# 環境変数 (CycloneDDS指定)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ROS2 setup
SHELL ["/bin/bash", "-c"]
# RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# SLAM
RUN apt update && apt install -y \
  ros-galactic-navigation2 \
  ros-galactic-nav2-bringup \
  ros-galactic-slam-toolbox \
  python3-pip \
  vim \
  && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc && \
    echo "source /root/colcon_ws/install/setup.bash" >> ~/.bashrc


RUN python3 -m pip install \
    pymycobot \
    numpy \
    opencv-python \
    pyrealsense2 \
    ultralytics \

# RUN cd /root/ &&
#     mkdir git -p && \
#     cd git && \
#     git clone https://github.com/robocup-sml/eai_task_server && \
#     git clone https://github.com/robocup-sml/sml_messages && \
#     mkdir ../colcon_ws -p && \
#     cd ../colcon_ws && \
#     ln -s ../git/eai_task_server . && \
#    ln -s ../git/sml_messages .

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# デフォルトでbash起動
CMD ["bash"]
