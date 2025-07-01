#!/bin/bash
DIR=`dirname $0`
source $DIR/setup.sh
echo $kachaka_IP
# go to kachaka-api
pushd $DIR/../../../kachaka-api/tools/ros2_bridge
./start_bridge.sh $kachaka_IP
popd
