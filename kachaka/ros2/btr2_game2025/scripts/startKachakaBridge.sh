#!/bin/bash
DIR=`dirname $0`
source $DIR/setup.sh
echo $kachaka_IP
# go to kachaka-api
# pushd $DIR/../../../kachaka-api/tools/ros2_bridge
# pushd ~/$kachakaDIR
# ./start_bridge.sh $kachaka_IP
# popd
kachaka=`cd; find . -name "start_bridge.sh" -follow`
pushd ~
echo $kachaka
bash $kachaka $kachaka_IP
popd
