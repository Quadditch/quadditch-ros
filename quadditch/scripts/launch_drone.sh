#! /bin/bash

if [[ -z $1 ]]
then
    echo "Requires the MAV_SYS_ID of the target drone as argument. Exiting."
    exit
fi

HOSTNAME=$(hostname)
MAV_SYS_ID=$1

echo "===== CONFIGURATION =========================="
echo "Target drone MAV_SYS_ID: $MAV_SYS_ID"
echo "Device namespace:        $HOSTNAME"
echo "=============================================="

roslaunch quadditch pixhawk.launch \
        HOSTNAME:=$HOSTNAME \
        MAV_ID:=$MAV_SYS_ID

echo "Finished."
