#include "ros/ros.h"
#include "std_msgs/String.h"

char *str;
const char *in;

float value;

void chatterCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());

	in = msg->data.c_str();

	while(*in) {
		printf("%f\n", strtod(in, &str));
		printf("Remainder: %s\n" ,str);
		in = str;
		/* move past the non-digits */
		while(!isdigit(*in) && *in) in++;
	}

}


int main(int argc, char **argv){

	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

	ros::spin();

	return 0;
}

