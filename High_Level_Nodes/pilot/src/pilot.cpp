#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "pilot.h"

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "pilot");	//inits the driver

	/* Messages and services */

	ros::NodeHandle pilotN;

	/* Publish */

	/* Subscribe */

	//ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	ROS_INFO("Pilot Is Online");

	while (ros::ok()){
		ros::spin();
	}

	printf("Shutting Down\n");

	return 0;
}
