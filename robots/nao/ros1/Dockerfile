FROM ros:noetic

# 必要パッケージ
RUN apt update && apt install -y \
    python2 \
    python2-minimal \
    python2-dev \
    python-is-python2 \
    ros-noetic-tf \
    ros-noetic-tf2 \
    ros-noetic-naoqi-driver \
    ros-noetic-naoqi-bridge-msgs \
    iputils-ping \
    netcat \
    && rm -rf /var/lib/apt/lists/*

ENV PYTHONPATH=/nao/pynaoqi-python2.7-2.1.4.13-linux64
ENV LD_LIBRARY_PATH=/nao/pynaoqi-python2.7-2.1.4.13-linux64


# ROS setupを自動化
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# 作業ディレクトリ
WORKDIR /root

# デフォルトコマンド
CMD ["/bin/bash"]
