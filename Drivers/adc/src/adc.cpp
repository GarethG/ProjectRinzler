#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "roboard.h"
#include "ad79x8.h"


#include "adc.h"


int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "adc");	//inits the driver

	/* Messages and services */

	ros::NodeHandle adcN;

	/* Publish */

	/* Subscribe */

	ROS_INFO("ADC Online");

	//ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	while (ros::ok()){
		ros::spin();
	}

	printf("Shutting Down\n");

	return 0;
}
