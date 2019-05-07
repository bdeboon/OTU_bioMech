#!/bin/bash
#$1 is the IP of the master
if [  -z "$2" ] # Check if argument exists
then
	echo "Missing IP Arguments: USEAGE : ./meca_falcon_teleop_master.sh <master_ip> <slave_ip>"
else
	echo "Starting Teleop Master"

	export ROS_IP=$1 #hostname of master
	export ROS_MASTER_URI=http://$1:11311 #Hostname of master
	export ROS_HOSTNAME=$2 

	rosrun meca500 meca_falcon.py
fi
