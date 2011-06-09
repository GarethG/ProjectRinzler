#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "pidRamp.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "pid");

	/* Messages and Services */

	ros::NodeHandle pidRampN;

	/* Publish */

	ros::Publisher slewRateMsg = pidRampN.advertise<std_msgs::Float32>("slewRate", 100);
	std_msgs::Float32 slewRate;

	/* Subscribe */

	ros::Subscriber sub = pidRampN.subscribe("pidRate", 100, rateCallback);

	ros::Rate loop_rate(50);

	ROS_INFO("PID Rate Online");

	while(ros::ok()){

		ros::spinOnce();

		slewRate.data = slewer();

		slewRateMsg.publish(slewRate);

		loop_rate.sleep();

	}

	printf("Shutting Down\n");

	return 0;
}


void rateCallback(const std_msgs::Float32::ConstPtr& pidRate){
	targetRate = pidRate->data;
	return;
}

float slewer(void){
	
	if(first){
		currentRate = targetRate;
		first = 0;
	}

	if(targetRate > currentRate){
		currentRate += CATCHRATE;
	}
	else if(targetRate < currentRate){
		currentRate -= CATCHRATE;
	}

	if(currentRate > MAXSPEED){
		currentRate = MAXSPEED;
	}

	if(currentRate < MINSPEED){
		currentRate = MINSPEED;
	}

	return currentRate;
}
