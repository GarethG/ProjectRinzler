#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

float go;

void goCallback(const std_msgs::Float32::ConstPtr& goNode);

int main(int argc, char **argv){

	ros::init(argc, argv, "waitNode");

	/* Messages and Services */

	ros::NodeHandle goN;

	/* Publish */

	/* Subscribe */

	ros::Subscriber sub1 = goN.subscribe("goNode", 100, goCallback);

	ros::Rate loop_rate(10);

	ROS_INFO("Wait Online");

	go = 0;

	while(ros::ok()){

		while(go != 1.0){

			ros::spinOnce();
		}

		ROS_INFO("I'm away");

		

		loop_rate.sleep();

	}





	printf("Shutting Down\n");

	return 0;
}

/*************************************************
** Returns the compass heading			**
*************************************************/

void goCallback(const std_msgs::Float32::ConstPtr& goNode){
	go = goNode->data;
	return;
}
