#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "pid.h"


int main(int argc, char **argv){

	float tmp;

	ros::init(argc, argv, "pid");

	/* Messages and Services */

	ros::NodeHandle pidN;

	if(!strcmp(argv[1],"heading")){

		/* Publish */

		ros::Publisher leftRateMsg = pidN.advertise<std_msgs::Float32>("leftRate", 100);
		ros::Publisher rightRateMsg = pidN.advertise<std_msgs::Float32>("rightRate", 100);

		std_msgs::Float32 leftRate;
		std_msgs::Float32 rightRate;

		/* Subscribe */

		ros::Subscriber sub1 = pidN.subscribe("compassHeading", 100, headingCallback);
		ros::Subscriber sub2 = pidN.subscribe("pilotHeading", 100, targetHeadingCallback);

		ros::Rate loop_rate(10);

		ROS_INFO("Heading PID Online");

		while(ros::ok()){

			ros::spinOnce();

			tmp = pid(targetHeading,heading);

			leftRate.data = tmp;
			rightRate.data = (tmp * -1.0);

			leftRate.data += FWDHACK;	//bit hacks but ensures we keep moving forwards even when balanced
			rightRate.data += FWDHACK;
			
			leftRateMsg.publish(leftRate);
			rightRateMsg.publish(rightRate);

			loop_rate.sleep();

		}

	}
	else if(!strcmp(argv[1],"depth")){
		
		/* Publish */

		ros::Publisher frontRateMsg = pidN.advertise<std_msgs::Float32>("frontRate", 100);
		ros::Publisher backDRateMsg = pidN.advertise<std_msgs::Float32>("backDRate", 100);

		std_msgs::Float32 frontRate;
		std_msgs::Float32 backDRate;

		/* Subscribe */

		ros::Subscriber sub1 = pidN.subscribe("svpDepth", 100, depthCallback);
		ros::Subscriber sub2 = pidN.subscribe("pilotDepth", 100, targetDepthCallback);

		ros::Rate loop_rate(10);

		ROS_INFO("Depth PID Online");

		while(ros::ok()){

			ros::spinOnce();

			tmp = pid(targetDepth,depth);

			if(depth > targetDepth){
				tmp = 0.0f;
			}
			else{
				tmp *= -1.0;
			}

			frontRate.data = tmp;

			if((targetDepth - depth) > FRONTTHRESH){
				backDRate.data = tmp;
				backDRateMsg.publish(backDRate);
			}
							

			frontRateMsg.publish(frontRate);

			loop_rate.sleep();

		}
	}
	else if(!strcmp(argv[1],"pitch")){

		/* Publish */

		ros::Publisher backPRateMsg = pidN.advertise<std_msgs::Float32>("backPRate", 100);
		std_msgs::Float32 backPRate;

		/* Subscribe */

		ros::Subscriber sub1 = pidN.subscribe("compassPitch", 100, pitchCallback);
		ros::Subscriber sub2 = pidN.subscribe("pilotPitch", 100, targetPitchCallback);

		ros::Rate loop_rate(10);

		ROS_INFO("Pitch PID Online");

		while(ros::ok()){

			ros::spinOnce();

			backPRate.data = pid(targetPitch,pitch);

			if(backPRate.data < 0.0){
				backPRate.data = 0.0;
			}

			backPRateMsg.publish(backPRate);

			loop_rate.sleep();

		}
	}
	else{
		ROS_ERROR("Please Enter 'heading', 'depth' or 'pitch'. Alternatively program some software for %s.",argv[1]);
	}




	printf("Shutting Down %s\n",argv[1]);

	return 0;
}


void headingCallback(const std_msgs::Float32::ConstPtr& compassHeading){
	heading = compassHeading->data;
	return;
}

void targetHeadingCallback(const std_msgs::Float32::ConstPtr& pilotHeading){
	targetHeading = pilotHeading->data;
	return;
}

void pitchCallback(const std_msgs::Float32::ConstPtr& compassPitch){
	pitch = compassPitch->data;
	return;
}

void targetPitchCallback(const std_msgs::Float32::ConstPtr& pilotPitch){
	targetPitch = pilotPitch->data;
	return;
}

void depthCallback(const std_msgs::Float32::ConstPtr& svpDepth){
	depth = svpDepth->data;
	return;
}

void targetDepthCallback(const std_msgs::Float32::ConstPtr& pilotDepth){
	targetDepth = pilotDepth->data;
	return;
}

float correctError(float error){
	if(error > 180.0){
		error -= 360.0;
	}
	else if(error < -180.0){
		error += 360.0;
	}
	
	return error;
}

float p(float value, float targetValue){
	float error;

	error = targetValue - value;	//error is target - actual

	error = correctError(error);

	error *= KP;				//multiply by constant

	return error;
}

float pd(float value, float targetValue){
	float error,derivative,lastError = 0.0f;

	error = targetValue - value;

	error = correctError(error);
	
	derivative = lastError - error;

	lastError = error;

	error *= KP;

	derivative *= KD;

	error += derivative;
	
	return error;
}

float pi(float value, float targetValue){
	float error, lastOut = 0.0f, lastError = 0.0f,output,tmp1,tmp2;

	error = targetValue - value;

	error = correctError(error);

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

float pid(float value, float targetValue){
	float error, lastOut = 0.0f, lastError = 0.0f, lastError2 = 0.0f,output,tmp1,tmp2,tmp3;

	error = targetValue - value;

	error = correctError(error);

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
