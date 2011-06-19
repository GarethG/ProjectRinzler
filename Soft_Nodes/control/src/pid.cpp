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

	/* Choose between heading, depth and pitch control */

	if(!strcmp(argv[1],"heading")){

		/* Publish */

		ros::Publisher leftRateMsg = pidN.advertise<std_msgs::Float32>("leftRate", 100);
		ros::Publisher rightRateMsg = pidN.advertise<std_msgs::Float32>("rightRate", 100);

		std_msgs::Float32 leftRate;
		std_msgs::Float32 rightRate;

		/* Subscribe */

		ros::Subscriber sub1 = pidN.subscribe("compassHeading", 100, headingCallback);
		ros::Subscriber sub2 = pidN.subscribe("pilotHeading", 100, targetHeadingCallback);
		ros::Subscriber sub3 = pidN.subscribe("pilotSpeed", 100, speedCallback);
		ros::Subscriber sub4 = pidN.subscribe("goNode", 100, goCallback);

		ros::Rate loop_rate(10);

		ROS_INFO("Heading PID Online");

		while(ros::ok()){

			while(go != 1.0){

				ros::spinOnce();
			}

			ros::spinOnce();

			tmp = pid(targetHeading,heading);	//get PID value

			leftRate.data = tmp;			//left does opposite of right
			rightRate.data = (tmp * -1.0);

			leftRate.data += speed;	//bit hacks but ensures we keep moving forwards even when balanced
			rightRate.data += speed;
			
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
		ros::Subscriber sub3 = pidN.subscribe("goNode", 100, goCallback);

		ros::Rate loop_rate(10);

		ROS_INFO("Depth PID Online");

		while(ros::ok()){
			
			while(go != 1.0){

				ros::spinOnce();
			}

			ros::spinOnce();

			tmp = pid(targetDepth,depth);

			if(depth > targetDepth){	//if we are too deep don't turn on the motors, use bouyancy
				tmp = 0.0f;
			}
			else{
				tmp *= -1.0;
			}

			frontRate.data = tmp;

			if((targetDepth - depth) > FRONTTHRESH){	//if we are really far off target
				backDRate.data = tmp;			//add rear motor power
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
		ros::Subscriber sub3 = pidN.subscribe("goNode", 100, goCallback);

		ros::Rate loop_rate(10);

		ROS_INFO("Pitch PID Online");

		while(ros::ok()){

			while(go != 1.0){

				ros::spinOnce();
			}

			ros::spinOnce();

			backPRate.data = pid(targetPitch,pitch);

			if(backPRate.data < 0.0){	//if the nose is high
				backPRate.data = 0.0;	//use bouyancy
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

/*************************************************
** Returns the go signal			**
*************************************************/

void goCallback(const std_msgs::Float32::ConstPtr& goNode){
	go = goNode->data;
	return;
}

/*************************************************
** Returns the pilots speed			**
*************************************************/

void speedCallback(const std_msgs::Float32::ConstPtr& pilotSpeed){
	speed = pilotSpeed->data;
	return;
}

/*************************************************
** Returns the compass heading			**
*************************************************/

void headingCallback(const std_msgs::Float32::ConstPtr& compassHeading){
	heading = compassHeading->data;
	return;
}

/*************************************************
** Returns the target heading			**
*************************************************/

void targetHeadingCallback(const std_msgs::Float32::ConstPtr& pilotHeading){
	targetHeading = pilotHeading->data;
	return;
}

/*************************************************
** Returns the compass pitch			**
*************************************************/

void pitchCallback(const std_msgs::Float32::ConstPtr& compassPitch){
	pitch = compassPitch->data;
	return;
}

/*************************************************
** Returns the target pitch			**
*************************************************/

void targetPitchCallback(const std_msgs::Float32::ConstPtr& pilotPitch){
	targetPitch = pilotPitch->data;
	return;
}

/*************************************************
** Returns the svp depth			**
*************************************************/

void depthCallback(const std_msgs::Float32::ConstPtr& svpDepth){
	depth = svpDepth->data;
	return;
}

/*************************************************
** Returns the target depth			**
*************************************************/

void targetDepthCallback(const std_msgs::Float32::ConstPtr& pilotDepth){
	targetDepth = pilotDepth->data;
	return;
}

/*************************************************
** Performed to correct the error as without	**
** this we will be often turning 270 degrees to	**
** the right instead of 90 degrees to the left	**
*************************************************/

float correctError(float error){
	if(error > 180.0){
		error -= 360.0;
	}
	else if(error < -180.0){
		error += 360.0;
	}
	
	return error;
}

/*************************************************
** P only control				**
*************************************************/

float p(float value, float targetValue){
	float error;

	error = targetValue - value;	//error is target - actual

	error = correctError(error);

	error *= KP;				//multiply by constant

	return error;
}

/*************************************************
** P and D control				**
*************************************************/

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

/*************************************************
** P and I control				**
*************************************************/

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

/*************************************************
** PID control					**
*************************************************/

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
