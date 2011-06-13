#include <math.h>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "depthMod.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "depthMod");

	/* Messages and Services */

	ros::NodeHandle depthModN;

	/* Publish */

	ros::Publisher backMsg = depthModN.advertise<std_msgs::Float32>("backRate", 100);

	std_msgs::Float32 backRate;

	/* Subscribe */

	ros::Subscriber sub1 = depthModN.subscribe("backDRate", 100, depthCallback);
	ros::Subscriber sub2 = depthModN.subscribe("backPRate", 100, pitchCallback);

	ros::Rate loop_rate(10);

	while(ros::ok()){

		depthChange = 0;	//clear depth flag
		ros::spinOnce();

		if(depthChange){
			backRatePitch *= PITCHP;			//reduce power for pitch
			backRate.data = backRatePitch + backRateDepth;	//combine values
		}
		else{
			backRate.data = backRatePitch;
		}

		backMsg.publish(backRate);

		loop_rate.sleep();

	}

	printf("Shutting Down\n");

	return 0;
}

/*************************************************
** Get what the depth is doing to the motor	**
*************************************************/

void depthCallback(const std_msgs::Float32::ConstPtr& backDRate){
	backRateDepth = backDRate->data;
	backRateDepth *= DEPTHP;	//power attributed to depth
	depthChange = 1;		//raise depth flag
	return;
}

/*************************************************
** Get what the pitch is doing to the motor	**
*************************************************/

void pitchCallback(const std_msgs::Float32::ConstPtr& backPRate){
	backRatePitch = backPRate->data;
	return;
}
