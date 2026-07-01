#!/bin/bash
set -e

source /opt/ros/galactic/setup.bash

# repoがなければclone
for DIR in eai_task_server sml_messages; do
  if [ ! -d /root/git/$DIR ]; then
    cd /root/git
    git clone https://github.com/robocup-sml/$DIR
  fi
done
if [ ! -d /root/git/rcll ]; then
  cd /root/git
  git clone https://github.com/babytigers-r/rcll
fi

# symlink作成
mkdir -p /root/colcon_ws/src
for DIR in eai_task_server sml_messages; do
  if [ ! -L /root/colcon_ws/src/$DIR ]; then
    ln -s /root/git/$DIR /root/colcon_ws/src/$DIR
  fi
done

# patch
cd /root/git/eai_task_server
patch -u < /root/git/rcll/myAGV/eai_task_server.patch

# build
cd /root/colcon_ws
colcon build

exec "$@"

