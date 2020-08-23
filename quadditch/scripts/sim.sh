#!/bin/bash
cd "$(dirname $(readlink -f $0))/../modules/Firmware"
make px4_sitl gazebo
