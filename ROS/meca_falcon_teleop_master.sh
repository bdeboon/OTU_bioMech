#!/bin/bash
#$1 is the IP of the master
if [  -z "$1" ] # Check if argument exists
then
	echo "Missing Master IP Argument: USE : ./meca_falcon_teleop_master.sh <master_ip>" 
else
	echo "Starting Teleop Master"
	#This was needed for my desktop, may be a different file on yours
	sudo chmod 777 /dev/bus/usb/001/*
	#Load firmware onto Falcon (Needs to be run 3 times for some reason)
	cd src/libnifalcon/examples/findfalcons/
	findfalcons
	findfalcons
	findfalcons
	#Config Master Service
	export ROS_IP=$1 #hostname of master
	export ROS_MASTER_URI=http://$1:11311 #Hostname of master
	export ROS_HOSTNAME=$1 
	#Run ROS node
	rosrun ros_falcon joy
fi

 
