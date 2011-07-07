#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"

#define COUNTLIM 150

void pilotOkGoCallback(const std_msgs::UInt32::ConstPtr& pilotOkGo);
unsigned int go = 0;
int main(int argc, char **argv){

	unsigned int count = 0;

	ros::init(argc, argv, "halt");

	/* Messages and Services */

	ros::NodeHandle haltN;

	/* Publish */

	ros::Publisher pilotGoMsg = haltN.advertise<std_msgs::UInt32>("pilotGo", 100);

	std_msgs::UInt32 pilotGo;

	/* Subscribe */


	ros::Subscriber sub1 = haltN.subscribe("pilotOkGo", 	100, pilotOkGoCallback);
	ros::Rate loop_rate(30);

	ROS_INFO("Halt Ready");

	while(ros::ok()){
		ros::spinOnce();
		if(go && (count > COUNTLIM)){
			pilotGo.data = 1;
		}
		else if(go){
			pilotGo.data = 0;
			count++;
		}
		else{
			pilotGo.data = 0;
			count = 0;
		}
		
		pilotGoMsg.publish(pilotGo);
		loop_rate.sleep();

	}

	printf("Shutting Down\n");

	return 0;
}

/*************************************************
** Returns the go				**
*************************************************/

void pilotOkGoCallback(const std_msgs::UInt32::ConstPtr& pilotOkGo){
	go = pilotOkGo->data;
	return;
}
