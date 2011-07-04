#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    
    int count = 0;
	
	ros::init(argc, argv, "sonarSpammer");
	
	ros::NodeHandle n;
	
	ros::Publisher xMsg = n.advertise<std_msgs::Int32>("logX", 100);
	ros::Publisher yMsg = n.advertise<std_msgs::Int32>("logY", 100);
	ros::Publisher zMsg = n.advertise<std_msgs::Int32>("logZ", 100);
	ros::Publisher aMsg = n.advertise<std_msgs::String>("logAction", 100);
	
	std_msgs::Int32 logX;
	std_msgs::Int32 logY;
	std_msgs::Int32 logZ;
	std_msgs::String logAction;


	while (ros::ok())
	{

		logX.data = count;
		logY.data = count;
		logZ.data = count;
		logAction.data = "hello y'all";
		
		xMsg.publish(logX);
		yMsg.publish(logY);
		zMsg.publish(logZ);
		aMsg.publish(logAction);
	
		sleep(1);
		
		count ++;
	}
	
}
