#!/bin/bash

compile_px4_sitl(){
    export DONT_RUN=1
    export PX4_HOME_LAT=37.2229
    export PX4_HOME_LON=-80.4324
    export PX4_HOME_ALT=455.3
    cd $1
    make px4_sitl_default gazebo -j8
    return
}

git submodule update --init --recursive
FIRM_PATH="$(dirname $(dirname $(dirname $(realpath $BASH_SOURCE))))/Firmware"
source $(dirname $(dirname $FIRM_PATH))/devel/setup.bash
compile_px4_sitl $FIRM_PATH &
wait
source $FIRM_PATH/Tools/setup_gazebo.bash $FIRM_PATH $FIRM_PATH/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$FIRM_PATH:$FIRM_PATH/Tools/sitl_gazebo
