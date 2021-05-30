#!/bin/bash

if pidof QGroundControl > /dev/null; then
	echo "QGroundControl already running"
else
	QGroundControl.AppImage
	#cd "$(dirname $(readlink -f $0))/"
	#./QGroundControl.AppImage
fi
