#!/usr/bin/env bash
source /opt/ros/melodic/setup.bash
cd ../ 
source devel/setup.bash

remoteIP=$(cat /var/lib/misc/dnsmasq.leases | grep -e d4e -e D4E | cut -d ' ' -f 3; )

export ROS_MASTER_URI=http://$remoteIP:11311
export ROS_IP=10.42.0.1


echo ROS_IP=$ROS_IP  ROS_MASTER_URI=$ROS_MASTER_URI
roslaunch root_framework D4E_auto.launch

