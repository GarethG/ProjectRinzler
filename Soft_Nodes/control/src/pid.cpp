#include "ros/ros.h"
#include "std_msgs/String.h"

void spamCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->sMsg);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "pid");

	ros::NodeHandle n;

	ROS_INFO("PID Online");

	ros::Subscriber sub = n.subscribe("spam", 100, spamCallback);

	while(ros::ok()){

		ros::spin();

	}

	printf("Shutting Down\n");

	return 0;
}
