#!/bin/bash
cat << EOF
This script makes the environment for BabyTigers-R.
EOF

GAZEBO_RCLL=~/git/gazebo-rcll
BTR_CODE=~/git/rcll
REFBOX_DIR=~/rcll-refbox
FLAG=FALSE
# install files from git.
### for refbox
if [ ! -d $REFBOX_DIR ]; then
	echo "install RefBox to $REFBOX_DIR"
	mkdir -p $REFBOX_DIR
	pushd $REFBOX_DIR
	cd ..
	git clone https://github.com/robocup-logistics/rcll-refbox
	cd rcll-refbox
	make -j$("nproc") all
	echo "You shoud modify the config files at cfg directory."
	popd
fi

#
### for gazebo
if [ ! -d $GAZEBO_RCLL ]; then
        echo "install GAZEBO-RCLL to $GAZEBO_RCLL"
        mkdir -p $GAZEBO_RCLL
        pushd $GAZEBO_RCLL/../
	git config http.postBuffer 52428000
        git clone https://github.com/robocup-logistics/gazebo-rcll
        # cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
        # cmake --build build
        popd
	FLAG=TRUE
fi

if [ ! -d $BTR_CODE ]; then
	echo "install BTR codes to $BTR_CODE"
	mkdir -p $BTR_CODE
	pushd $BTR_CODE
	cd ..
	git clone https://github.com/babytigers-r/rcll
	FLAG=TURE
fi

if [ "$FLAG" = "TRUE" ]; then
	# add some files for BTR
        mkdir -p $GAZEBO_RCLL/models/btr
	for RMFILE in `ls $BTR_CODE/gazebo/btr/models/`; do 
		rm -rf $GAZEBO_RCLL/models/$RMFILE 
		rm -rf $GAZEBO_RCLL/models/carologistics/$RMFILE
	done
	ln -s $BTR_CODE/gazebo/btr/models/* $GAZEBO_RCLL/models/btr/
	ln -s $BTR_CODE/gazebo/btr/world/* $GAZEBO_RCLL/worlds/carologistics/
	for PLUGIN in motor odometry mps; do
                rm $GAZEBO_RCLL/plugins/src/plugins/$PLUGIN -r
                ln -s $BTR_CODE/gazebo/btr/plugins/src/plugins/$PLUGIN $GAZEBO_RCLL/plugins/src/plugins/
	done
	ln -s $GAZEBO_RCLL/plugins/src/plugins/puck $BTR_CODE/gazebo/btr/plugins/src/plugins/
	for FILE in ResetOdometryResponse.h ResetOdometryRequest.h ResetOdometry.h; do
                rm $GAZEBO_RCLL/plugins/src/plugins/odometry/$FILE
                ln -s /home/$USER/catkin_ws/devel/include/robotino_msgs/$FILE $GAZEBO_RCLL/plugins/src/plugins/odometry/
	done
        echo "include_directories(/home/$USER/catkin_ws/devel/include)" >> $GAZEBO_RCLL/plugins/src/plugins/odometry/CMakeLists.txt
	rm $GAZEBO_RCLL/CMakeLists.txt 
	ln -s $BTR_CODE/gazebo/btr/CMakeLists.txt $GAZEBO_RCLL/
	pushd $GAZEBO_RCLL
	cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
        cmake --build build
	popd
fi

