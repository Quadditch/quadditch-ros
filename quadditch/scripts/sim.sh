#!/bin/bash
cd "$(dirname $(readlink -f $0))/../../Firmware"
export PX4_HOME_LAT=37.2229
export PX4_HOME_LON=-80.4324
#export PX4_HOME_LAT=40.2912
#export PX4_HOME_LON=-76.6729
#export PX4_HOME_ALT=28.5
#make px4_sitl gazebo_typhoon_h480 -j8
make px4_sitl gazebo -j8
