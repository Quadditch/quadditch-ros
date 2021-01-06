#!/bin/bash

git submodule update --init --recursive
export DONT_RUN=1
cd "$(dirname $(dirname $(dirname $(realpath $0))))/Firmware"
make px4_sitl_default gazebo -j8
