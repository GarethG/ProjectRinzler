#include <math.h>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "pid.h"


int main(int argc, char **argv){

	ros::init(argc, argv, "pid");

	/* Messages and Services */

	ros::NodeHandle pidN;

	/* Publish */

	ros::Publisher pidRateMsg = pidN.advertise<std_msgs::Float32>("pidRate", 100);
	std_msgs::Float32 pidRate;

	/* Subscribe */

	ros::Subscriber sub1 = pidN.subscribe("compassHeading", 100, headingCallback);
	ros::Subscriber sub2 = pidN.subscribe("pilotHeading", 100, targetCallback);

	ros::Rate loop_rate(10);

	ROS_INFO("PID Online");

	while(ros::ok()){

		/*ros::spinOnce();

		printf("Heading: %.3f\n",heading);

		loop_rate.sleep();*/

		ros::spinOnce();

		//printf("Actual: %.3f Target: %.3f\n",heading,targetHeading);

		//printf("Motor: %.5f\n",pid());

		pidRate.data = pid();

		pidRateMsg.publish(pidRate);

		loop_rate.sleep();

	}

	printf("Shutting Down\n");

	return 0;
}


void headingCallback(const std_msgs::Float32::ConstPtr& compassHeading){
	heading = compassHeading->data;
	return;
}

void targetCallback(const std_msgs::Float32::ConstPtr& pilotHeading){
	targetHeading = pilotHeading->data;
	return;
}

float p(void){
	float error;

	error = targetHeading - heading;	//error is target - actual

	error *= KP;				//multiply by constant

	return error;
}

float pd(void){
	float error,derivative,lastError = 0.0f;

	error = targetHeading - heading;
	
	derivative = lastError - error;

	lastError = error;

	error *= KP;

	derivative *= KD;

	error += derivative;
	
	return error;
}

float pi(void){
	float error, lastOut = 0.0f, lastError = 0.0f,output,tmp1,tmp2;

	error = targetHeading - heading;

	tmp1 = error - lastError;
	tmp1 *= KP;

	tmp2 = (error + lastError) / 2.0f;
	tmp2 *= KI;

	output = lastOut + tmp1 + tmp2;

	output = fmaxf(output,MAXVAL);
	output = fminf(output,MINVAL);

	lastOut = output;

	lastError = error;

	return error;
}

float pid(void){
	float error, lastOut = 0.0f, lastError = 0.0f, lastError2 = 0.0f,output,tmp1,tmp2,tmp3;

	error = targetHeading - heading;

	tmp1 = error - lastError;
	tmp1 *= KP;

	tmp2 = (error + lastError) / 2.0f;
	tmp2 *= KI;

	tmp3 = error - (2.0f * lastError) + lastError2;
	tmp3 *= KD;

	output = lastOut + tmp1 + tmp2 + tmp3;

	lastOut = output;

	lastError2 = lastError;
	lastError = error;

	return output;
}
