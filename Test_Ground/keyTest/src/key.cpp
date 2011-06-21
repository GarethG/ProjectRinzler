#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"


int main(int argc, char **argv){

	char str[80];

	ros::init(argc, argv, "key");

	/* Messages and Services */

	ros::NodeHandle keyN;

	/* Publish */

	/* Subscribe */

	//ros::Rate loop_rate(1);

	ROS_INFO("Key Online");

	while(ros::ok()){
		printf("Enter a key\n");
		scanf("%79s", str);
		printf("You pressed %s\n",str);
		printf("I care about %c\n",str[0]);
		//loop_rate.sleep();

	}

	printf("Shutting Down\n");

	return 0;
}
