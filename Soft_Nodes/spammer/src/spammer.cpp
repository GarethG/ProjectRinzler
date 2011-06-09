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

	/*Advertises our various messages*/

	ros::Publisher spamMsg = n.advertise<std_msgs::Float32>("pilotHeading", 100);

	/*Sets up the message structures*/

	std_msgs::Float32 sMsg;

	ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	ROS_INFO("Spammer Online");

	while (ros::ok()){		
		
		sMsg.data = 0.0f;

		/*Below here we publish our readings*/

		spamMsg.publish(sMsg);

		/*Have a snooze*/

		ros::spinOnce();

		loop_rate.sleep();

	}

	printf("Shutting Down\n");

	return 0;
}
