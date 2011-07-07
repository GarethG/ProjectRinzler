#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

int main(int argc, char **argv)
{

	int input;
		
	//ham
	ros::init(argc, argv, "sonarSpammer");
	
	ros::NodeHandle n;
	
	ros::Publisher pub1 = n.advertise<std_msgs::Int32>("sonarCmd", 100);
	
	ros::Publisher pub2 = n.advertise<std_msgs::Int32>("sonarRange", 100);
	
	std_msgs::Int32 sonarCmd;
	std_msgs::Int32 sonarRange;

	
	while (ros::ok())
	{
			
		printf("Input Sonar Range: ");
		if(scanf("%d", &input) != 0)	
			sonarRange.data = input;
		else
		{
			printf("No range set, defaulted to 75\n");
			sonarRange.data = 75;
		}

		printf("Input 1 for Stare, 0 for Scan: ");
		if(scanf("%d", &input) != 0)	
			sonarCmd.data = input;
		else
		{
			printf("No Setting set, going to scan\n");
			sonarCmd.data = 0;
		}

		pub1.publish(sonarCmd);
		pub2.publish(sonarRange);
		
		ros::spinOnce();
		
		usleep(2000);
	}
	
}
