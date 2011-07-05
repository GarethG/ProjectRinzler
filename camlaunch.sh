#!/bin/bash

sudo -i

source /opt/ros/diamondback/setup.bash
export ROS_PACKAGE_PATH=/home/uwesub/ProjectRinzler:$ROS_PACKAGE_PATH
export ROS_MASTER_URI=http://192.168.2.10:11311
export ROS_IP=192.168.2.20
export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb ! ffmpegcolorspace"

sleep 2

roslaunch /home/uwesub/ProjectRinzler/Camera.launch
