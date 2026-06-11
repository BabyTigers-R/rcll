FROM osrf/ros:galactic-desktop

# 環境変数 (CycloneDDS指定)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# パッケージのインストール
RUN apt update && apt install -y \
  git \
  openssh-client \
  && rm -rf /var/lib/apt/lists/*

# ROS2 setup
SHELL ["/bin/bash", "-c"]
# RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# SLAM
RUN apt update && apt install -y \
  ros-galactic-joy ros-galactic-teleop-twist-joy \
  ros-galactic-navigation2 \
  ros-galactic-nav2-bringup \
  ros-galactic-slam-toolbox \
  && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# デフォルトでbash起動
CMD ["bash"]
