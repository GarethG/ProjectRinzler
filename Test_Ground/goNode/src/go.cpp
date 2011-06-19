#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"


int main(int argc, char **argv){

	int counter;

	ros::init(argc, argv, "goNode");

	/* Messages and Services */

	ros::NodeHandle go2N;

	/* Publish */

	ros::Publisher goMsg = go2N.advertise<std_msgs::Float32>("goNode", 100);

	std_msgs::Float32 goNode;

	/* Subscribe */

	ros::Rate loop_rate(1);

	ROS_INFO("GO Online");

	counter = 10;

	while(ros::ok()){

		if(counter > 0){

			ROS_INFO("We go in %d seconds",counter);

			counter--;

			if(counter == 0){
				goNode.data = 1.0;
				goMsg.publish(goNode);	
			}
			else{
				goNode.data = 0;
				goMsg.publish(goNode);		
			}
		}


		loop_rate.sleep();

	}





	printf("Shutting Down\n");

	return 0;
}
