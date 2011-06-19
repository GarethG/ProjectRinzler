#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/io.h>
#include <stdlib.h>

#include "ros/ros.h"

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "iopl");	//inits the driver

	if(iopl(3) < 0){
		ROS_INFO("did not work");
		return -1;
	}    
	else{
		ROS_INFO("worked");
	}

	return 0;
}
