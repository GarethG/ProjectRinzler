#!/bin/bash

sudo -i

source /opt/ros/diamondback/setup.bash
export ROS_PACKAGE_PATH=/home/uwesub/ProjectRinzler:$ROS_PACKAGE_PATH
export ROS_MASTER_URI=http://192.168.2.10:11311
export ROS_IP=192.168.2.10

roslaunch /home/uwesub/ProjectRinzler/Rinzler.launch
