#! /bin/bash

ping_device ()
{
    IPADDR=$(dig $1 +short)
    if [[ -z $IPADDR ]]
    then
        echo "Couldn't resolve hostname \"$1\"."
        return
    fi
    printf "%-55s" "Pinging $1 ($IPADDR)... "
    stuff=$(ping $IPADDR -W 1 -c 1)
    if [[ 0 -eq $? ]]
    then
        echo "Success."
    else
        echo -e "FAILURE."
    fi
}

echo "Network ping at $(date +%Y-%m-%dT%T)"

ping_device access-point-1
ping_device access-point-2
ping_device rosmaster

printf "%-55s" "Checking if ROS master is up..."
nc -zvw10 rosmaster 11311 >/dev/null 2>/dev/null
if [[ 0 -eq $? ]]
then
    echo "Success."
else
    echo "FAILURE."
fi

ping_device uav5
ping_device uav7

