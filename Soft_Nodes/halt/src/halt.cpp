#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
void goTCallback(const std_msgs::UInt32::ConstPtr& pilotTGo);
unsigned int go = 0;
int main(int argc, char **argv){



	ros::init(argc, argv, "halt");

	/* Messages and Services */

	ros::NodeHandle haltN;

	/* Publish */

	ros::Publisher pilotGoMsg = haltN.advertise<std_msgs::UInt32>("pilotGo", 100);

	std_msgs::UInt32 pilotGo;

	/* Subscribe */

	ros::Subscriber sub1 = haltN.subscribe("pilotTGo", 	100, goTCallback);

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

void goTCallback(const std_msgs::UInt32::ConstPtr& pilotTGo){
	go = pilotTGo->data;
	return;
}
