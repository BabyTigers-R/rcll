FROM --platform="linux/x86_64" tiryoh/ros2-desktop-vnc:humble-amd64
LABEL maintainer="Wataru UEMURA<wataru@kdel.org>"

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y openssh-server

### for ssh server
RUN mkdir /var/run/sshd && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/g' /etc/ssh/sshd_config && \
    sed -i 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' /etc/pam.d/sshd && \
    head -n `expr \`wc /entrypoint.sh|awf print {$1}\` -1 ` /entrypoint.sh > /start-ssh.sh && \
    echo "sudo /etc/init.d/ssh start" >> /start-ssh.sh && \
    tail -n 1 /entrypoint.sh >> /start-ssh.sh && \
    cp /start-ssh.sh /entrypoint.sh && \
    rm /start-ssh.sh

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq \
                     net-tools git gcc build-essential vim screen ccache && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq \ 
                     libmodbus-dev \
                     protobuf-compiler libprotobuf-dev libprotoc-dev \
                     libboost-all-dev libmodbus-dev \
                     libglibmm-2.4-dev libgtkmm-3.0-dev \
                     libncursesw5-dev libyaml-cpp-dev libavahi-client-dev \
                     libssl-dev libgecode-dev \
                     libncurses5-dev cmake-curses-gui \
                     libmbedtls-dev g++ libtinfo5 \
                     libspdlog-dev libspdlog1 && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq \
      gnome && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq \
      python3-pip python3-usb && \
    rm -rf /var/lib/apt/lists/*

# RUN apt-get update -q && \
#     apt-get upgrade -yq && \
#     DEBIAN_FRONTEND=noninteractive apt-get install -yq \
#                      ros2-humble-nav2-common && \
#     rm -rf /var/lib/apt/lists/*

RUN dpkg --add-architecture i386 && \
    apt-get update -q && \ 
    DEBIAN_FRONTEND=noninteractive apt-get install -yq \
      wine32 gosu && \
    rm -rf /var/lib/apt/lists/* && \
    echo "wine32"

### for robotino sim demo
RUN mkdir -p /home/ryukoku/Downloads && \ 
    cd /home/ryukoku/Downloads && \
    wget https://doc.openrobotino.org/download/RobotinoSim/RobotinoSimDemo-1.4.1.exe

RUN mkdir -p /home/ryukoku && \
    echo '#!/bin/bash' > /home/ryukoku/robotinoSimDemo.sh && \
    echo 'if [ -d ~/.wine/drive_c/Program\ Files/Didactic/RobotinoSim\ Demo ] ; then wine ~/.wine/drive_c/Program\ Files/Didactic/RobotinoSim\ Demo/bin/robotinosim.exe; else cd ~/Downloads && wine RobotinoSimDemo-1.4.1.exe; fi' >> /home/ryukoku/robotinoSimDemo.sh && \
    chmod +x /home/ryukoku/robotinoSimDemo.sh &&\
    echo "robotinoSimDemo"

ENV USER=ryukoku
ARG PASSWORD=elec
