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
    
	float bearing = 0.0;
	//ham
	ros::init(argc, argv, "sonarSpammer");
	
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("sonarBinsArr", 100);
	
	ros::Publisher sonarBearingMsg = n.advertise<std_msgs::Float32>("sonarBearing", 100);
	
	std_msgs::Int32MultiArray sonarBinsArr;
	std_msgs::Float32 sonarBearing;

	
	while (ros::ok())
	{
		if(bearing == 6399.0)
			bearing = 0.0;
			
		sonarBinsArr.data.clear();
		for (int i = 0; i < 90; i++)
		{
			sonarBinsArr.data.push_back(rand() % 255);
		}
		sonarBearing.data = bearing;

		pub.publish(sonarBinsArr);
		sonarBearingMsg.publish(sonarBearing);
		
		printf("%f \n", bearing);
		
		ros::spinOnce();
		
		bearing = bearing + 1.0;
		usleep(2000);
	}
	
}
