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

# Python 3.10
#RUN apt update && apt install -y software-properties-common && \
#    add-apt-repository ppa:deadsnakes/ppa && \
#    apt update && \
#    apt install -y python3.10 python3.10-venv python3.10-distutils && \
#    curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10 && \
#    rm -rf /var/lib/apt/lists/* && \
RUN apt update && apt install -y \
    build-essential wget curl \
    libssl-dev zlib1g-dev \
    libbz2-dev libreadline-dev libsqlite3-dev \
    libncurses5-dev libncursesw5-dev \
    xz-utils tk-dev libffi-dev && \
    wget https://www.python.org/ftp/python/3.10.13/Python-3.10.13.tgz && \
    tar xzf Python-3.10.13.tgz && \
    cd Python-3.10.13 && \
    ./configure --enable-optimizations && \
    make -j$(nproc) && \
    make altinstall && \
    cd / && rm -rf Python-3.10.13* && \
    python3.10 -m venv venv310

RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc && \
    echo "source venv310/bin/activate" >> ~/.bashrc

# デフォルトでbash起動
CMD ["bash"]
