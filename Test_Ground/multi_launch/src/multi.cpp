#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"


int main(int argc, char **argv){

	ros::init(argc, argv, "multi");

	ros::NodeHandle pidN;

	ros::Rate loop_rate(1);

	while(ros::ok()){
		printf("We started with %s\n",argv[1]);
		loop_rate.sleep();
	}

	return 0;
}
