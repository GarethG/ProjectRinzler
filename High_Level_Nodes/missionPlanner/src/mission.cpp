#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "mission.h"

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "mission");	//inits the driver

	/* Messages and services */

	ros::NodeHandle missionN;

	/* Publish */

	/* Subscribe */

	//ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	ROS_INFO("Mission Planner Is Online");

	while (ros::ok()){
		ros::spin();
	}

	printf("Shutting Down\n");

	return 0;
}
