#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"


int main(int argc, char **argv){

	char c='X';
	float newHeading=0.0, newDepth=0.0, newPitch=0.0, newSpeed=0.0;

	ros::init(argc, argv, "key");

	/* Messages and Services */

	ros::NodeHandle keyN;

	/* Publish */

	ros::Publisher keyGoMsg = keyN.advertise<std_msgs::UInt32>("keyGo", 100);
	ros::Publisher keyHeadingMsg = keyN.advertise<std_msgs::Float32>("keyHeading", 100);
	ros::Publisher keyDepthMsg = keyN.advertise<std_msgs::Float32>("keyDepth", 100);
	ros::Publisher keyPitchMsg = keyN.advertise<std_msgs::Float32>("keyPitch", 100);
	ros::Publisher keySpeedMsg = keyN.advertise<std_msgs::Float32>("keySpeed", 100);

	/*Sets up the message structures*/

	std_msgs::UInt32 keyGo;
	std_msgs::Float32 keyHeading;
	std_msgs::Float32 keyDepth;
	std_msgs::Float32 keyPitch;
	std_msgs::Float32 keySpeed;

	keyGo.data = 0;
	keyHeading.data = 0.0;
	keyDepth.data = 0.0;
	keyPitch.data = 0.0;
	keySpeed.data = 0.0;

	/* Subscribe */

	ROS_INFO("Key Online");

	while(ros::ok()){

		if(c!='Y'){
			ROS_INFO("Would you like to go yet? Y to go.");
			scanf("%c",&c);
			keyGo.data = 1;
		}
		if(c=='Y'){		
			ROS_INFO("Please Enter Your Heading (Float)");
			scanf("%f", &newHeading);
			if((newHeading > 359.999) || (newHeading < 0.0)){
				ROS_ERROR("Invalid Heading");
			}
			else{
				ROS_INFO("New Heading is %.3f",newHeading);
				keyHeading.data = newHeading;
			}
			ROS_INFO("Please Enter Your Depth (Float)");
			scanf("%f", &newDepth);
			if((newDepth < 0.0) || (newDepth > 40.0)){
				ROS_ERROR("Invalid Depth");
			}
			else{
				ROS_INFO("New Depth is %.3f",newDepth);
				keyDepth.data = newDepth;
			}
			ROS_INFO("Please Enter Your Pitch (Float)");
			scanf("%f", &newPitch);
			if((newPitch < -10.0) || (newPitch > 10.0)){
				ROS_ERROR("Invalid Pitch");
			}
			else{
				ROS_INFO("New Pitch is %.3f",newPitch);
				keyPitch.data = newPitch;
			}
			ROS_INFO("Please Enter Your Speed (Float)");
			scanf("%f", &newSpeed);
			if((newSpeed < -10.0) || (newSpeed > 20.0)){
				ROS_ERROR("Invalid Speed");
			}
			else{
				ROS_INFO("New Speed is %.3f",newSpeed);
				keySpeed.data = newSpeed;
			}

			keyGoMsg.publish(keyGo);
			keyHeadingMsg.publish(keyHeading);
			keyDepthMsg.publish(keyDepth);
			keyPitchMsg.publish(keyPitch);
			keySpeedMsg.publish(keySpeed);

			ros::spinOnce();

		}
	}

	printf("Shutting Down\n");

	return 0;
}
