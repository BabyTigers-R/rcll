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
  && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# デフォルトでbash起動
CMD ["bash"]

# 追加のセットアップやコマンドはここに記述