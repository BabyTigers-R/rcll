#!/bin/bash
cat << EOF
This script makes the environment for BabyTigers-R.
EOF

GAZEBO_RCLL=~/git/gazebo-rcll
BTR_2023=~/git/btr2023
BTR_CODE=~/git/btr2023/bordeaux2023
REFBOX_DIR=~/rcll-refbox

# install files from git.
if [ ! -d $GAZEBO_RCLL ]; then
	echo "install GAZEBO-RCLL to $GAZEBO_RCLL"
	mkdir -p $GAZEBO_RCLL
	pushd $GAZEBO_RCLL
	cd ..
	git clone https://github.com/robocup-logistics/gazebo-rcll
	cd gazebo-rcll
	cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
	cmake --build build
	popd
fi

if [ ! -d $REFBOX_DIR ]; then
	echo "install RefBox to $REFBOX_DIR"
	mkdir -p $REFBOX_DIR
	pushd $REFBOX_DIR
	cd ..
	git clone https://github.com/robocup-logistics/rcll-refbox
	cd rcll-refbox
	make -j4 all
	echo "You shoud modify the config files at cfg directory."
	popd
fi

if [ ! -d $BTR_2023 ]; then
	echo "install BTR2023 codes to $BTR_2023"
	mkdir -p $BTR_2023
	pushd $BTR_2023
	cd ..
	git clone https://github.com/wadaru/btr2023
	# replace some files for BTR2023
	ln -s $BTR_CODE/gazebo/btr/models $GAZEBO_RCLL/models/btr
	ln -s $BTR_CODE/gazebo/btr/world $GAZEBO_RCLL/worlds/btr
	echo "please change the GAZEBO_WORLD_PATH to $GAZEBO_RCLL/worlds/btr/*"
	for PLUGIN in motor odometry; do
		rm $GAZEBO_RCLL/plugins/src/plugins/$PLUGIN -r
		ln -s $BTR_CODE/gazebo/btr/plugins/src/plugins/$PLUGIN $GAZEBO_RCLL/plugins/src/plugins/
	done
	rm $GAZEBO_RCLL/CMakeLists.txt 
	ln -s $BTR_CODE/gazebo/btr/CMakeLists.txt $GAZEBO_RCLL/
	popd
fi

