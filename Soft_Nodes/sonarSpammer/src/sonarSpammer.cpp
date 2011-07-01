#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"

int main(int argc, char **argv)
{
    
	
	ros::init(argc, argv, "sonarSpammer");
	
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("sonarBinsArr", 100);

	while (ros::ok())
	{
		std_msgs::Int32MultiArray sonarBinsArr;
		sonarBinsArr.data.clear();
		for (int i = 0; i < 90; i++)
		{
			sonarBinsArr.data.push_back(rand() % 255);
		}

		pub.publish(sonarBinsArr);
		ROS_INFO("I published something!");
		ros::spinOnce();
		
		sleep(2);
	}
	
}
