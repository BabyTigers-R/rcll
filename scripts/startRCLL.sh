#!/bin/bash
##
USER_NAME=`id -u -n`
HOME_DIR="/home/$USER_NAME"
REFBOX_DIR="$HOME_DIR/rcll-refbox/bin"
BTR_DIR="$HOME_DIR/git/rcll"
# GAME_DIR="bordeaux2023"
GAME_DIR="."
TERM="gnome-terminal --tab -- bash -c "
NEWTERM="gnome-terminal --window --maximize -- bash -c "

SCRIPT_DIR="$BTR_DIR/$GAME_DIR/scripts"
PYTHON_DIR="$BTR_DIR/$GAME_DIR/python"
PICTURES_DIR="$BTR_DIR/$GAME_DIR/pictures"
RPLIDAR_DIR="$HOME_DIR/catkin_ws/src/rplidar_ros/launch"
ROBOTINO_DIR="$HOME_DIR/catkin_ws/src/robotino_node/launch"

for PROGNAME in roscore gzclient gzserver gazebo; do
	killall $PROGNAME 2> /dev/null
done

sudo chmod 777 /dev/ttyUSB? /dev/ttyACM? /dev/video? 2> /dev/null

pushd `dirname $0`
if [ ! -d $PICTURES_DIR ]; then
	ls $PICTURES_DIR
	mkdir $PICTURES_DIR
fi

if [ "$1" = "help" -o "$1" = "-h" -o "$1" = "--help" ]; then
	echo << EOF
usage: $0 [OPTION] [ROBOTNUMBER]
 -h, --help, help	give this help list
 -g, --gazebo, gazebo   run gazebo simulator environment using ROBOTNUMBER as its robot number
EOF
	exit
fi
if [ "$1" = "gazebo" -o "$1" = "-g" -o "$1" = "--gazebo" ]; then
	GAZEBO="true"
fi


COMMAND="$COMMAND $TERM \\\"roscore\\\";"

COMMAND="$COMMAND $TERM \\\"echo rosRcllRefBoxNetwork.sh; sleep 7; cd $SCRIPT_DIR; bash -c ./rosRcllRefBoxNetwork.sh\\\";"
if [ "${GAZEBO}" ]; then
	COMMAND="$COMMAND $TERM \\\"echo refbox; cd $REFBOX_DIR; ./llsf-refbox\\\";"
	COMMAND="$COMMAND $TERM \\\"echo refbox-shell; cd $REFBOX_DIR; ./llsf-refbox-shell\\\";"
	COMMAND="$COMMAND $TERM \\\"echo gazebo; sleep 3; rosrun gazebo_ros gazebo $GAZEBO_WORLD_PATH\\\";"
	for ROBOTNO in 1 2 3; do
		COMMAND="$COMMAND $TERM \\\"echo btr_rplidar.py gazebo $ROBOTNO; sleep 7; cd $PYTHON_DIR; python3 ./btr_rplidar.py gazebo $ROBOTNO\\\";"
		COMMAND="$COMMAND $TERM \\\"echo btr_gazebo_camera.py gazezbo $ROBOTNO; sleep 7; cd $PYTHON_DIR; python3 ./btr_gazebo_camera.py gazebo $ROBOTNO; bash\\\";"
		COMMAND="$COMMAND $TERM \\\"echo btr_aruco.py gazebo $ROBOTNO; sleep 8; cd $PYTHON_DIR; python3 ./btr_aruco.py gazebo $ROBOTNO; bash\\\";"
	done
else
	COMMAND="$COMMAND $TERM \\\"echo robotino_node; sleep 1; cd $ROBOTINO_DIR; roslaunch robotino_node.launch hostname:=127.0.1.1; bash\\\";"
	COMMAND="$COMMAND $TERM \\\"echo rplidar.launch; sleep 1; cd $RPLIDAR_DIR; roslaunch rplidar_a3.launch; bash\\\";"
	COMMAND="$COMMAND $TERM \\\"echo btr_rplidar.py; sleep 2; cd $PYTHON_DIR; python3 ./btr_rplidar.py; bash\\\";"
	COMMAND="$COMMAND $TERM \\\"echo btr_camera.py; sleep 1; cd $PYTHON_DIR; python3 ./btr_camera.py $1 $2; bash\\\";"
	COMMAND="$COMMAND $TERM \\\"echo btr_aruco.py; sleep 2; cd $PYTHON_DIR; python3 ./btr_aruco.py $1 $2; bash\\\";"
	COMMAND="$COMMAND $TERM \\\"echo btr_myPalletizer_ros.py; sleep 2; cd $PYTHON_DIR; python3 ./btr_myPalletizer_ros.py $1 $2; bash\\\";"
fi

COMMAND="$NEWTERM \"$COMMAND\""
# echo $COMMAND
eval $COMMAND
popd
