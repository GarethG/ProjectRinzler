#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"


int main(int argc, char **argv){

	float go = 0.0;

	ros::init(argc, argv, "halt");

	/* Messages and Services */

	ros::NodeHandle haltN;

	/* Publish */

	ros::Publisher pilotGoMsg = haltN.advertise<std_msgs::Float32>("pilotGo", 100);

	std_msgs::Float32 pilotGo;

	/* Subscribe */

	ros::Rate loop_rate(30);

	ROS_INFO("Halt Ready");

	while(ros::ok()){
		ros::spinOnce();
		pilotGo.data = go;
		pilotGoMsg.publish(pilotGo);
		loop_rate.sleep();

	}

	printf("Shutting Down\n");

	return 0;
}

/*************************************************
** Returns the go				**
*************************************************/

void goTCallback(const std_msgs::Float32::ConstPtr& pilotTGo){
	go = pilotTGo->data;
	return;
}
