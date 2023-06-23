#!/usr/bin/env bash
source /opt/ros/melodic/setup.bash
# change to your location of catkinWS
#source /home/obs_uas/srcCode/MSC_WS/devel/setup.bash

remoteIP=$(cat /var/lib/misc/dnsmasq.leases | grep -e d4e -e D4E | cut -d ' ' -f 3; )
echo $remoteIP
export ROS_MASTER_URI=http://$remoteIP:11311
export ROS_IP=10.42.0.1

roslaunch mavros px4.launch fcu_url:=udp://0.0.0.0:14540@$remoteIP:14557 gcs_url:=udp://@10.42.0.1

