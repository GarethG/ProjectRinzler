#include <stdio.h>   
#include <unistd.h>  
#include <fcntl.h>   
#include <errno.h>   
#include <termios.h> 
#include <stdlib.h>
#include <math.h>
#include <string.h>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>


int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "spam");	//inits the driver
	ros::NodeHandle n;			//this is what ROS uses to connect to a node

	float out = 0.0f,out2 = 20.0f;
	unsigned int direc = 0, direc2 = 0;

	/*Advertises our various messages*/

	ros::Publisher spamMsg = n.advertise<std_msgs::Float32>("pilotHeading", 100);
	ros::Publisher spamMsg2 = n.advertise<std_msgs::Float32>("compassHeading", 100);
	ros::Publisher spamMsg3 = n.advertise<std_msgs::Float32>("pilotDepth", 100);
	ros::Publisher spamMsg4 = n.advertise<std_msgs::Float32>("svpDepth", 100);

	/*Sets up the message structures*/

	std_msgs::Float32 pilotHeading;
	std_msgs::Float32 compassHeading;
	std_msgs::Float32 pilotDepth;
	std_msgs::Float32 svpDepth;

	ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	ROS_INFO("Spammer Online");

	while (ros::ok()){	

		if(direc){
			out += 0.2;
			if(out > 10.0){
				direc = 0;
			}
		}
		else{
			out -= 0.2;
			if(out < -10.0){
				direc = 1;
			}
		}

		compassHeading.data = out;

		if(direc2){
			out2 += 0.2;
			if(out2 > 60.0){
				direc2 = 0; 
			}
		}
		else{
			out2 -= 0.2;
			if(out2 < 1.0){
				direc2 = 1;
			}
		}	

		svpDepth.data = out2;
		
		pilotHeading.data = 0.0f;
		pilotDepth.data = 30.0f;

		/*Below here we publish our readings*/

		spamMsg.publish(pilotHeading);		
		spamMsg2.publish(compassHeading);
		spamMsg3.publish(pilotDepth);		
		spamMsg4.publish(svpDepth);

		/*Have a snooze*/

		ros::spinOnce();

		loop_rate.sleep();

	}

	printf("Shutting Down\n");

	return 0;
}
