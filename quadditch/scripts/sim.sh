#!/bin/bash
cd "$(dirname $(readlink -f $0))/ardupilot/Tools/autotest"
./sim_vehicle.py -w -v ArduCopter -I 0 --custom-location="37.2229,-80.4324,611.0,300" --console --map
