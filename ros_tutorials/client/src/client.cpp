#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

char *str;
const char *in;

float value;

void chatterCallback(const std_msgs::Float32::ConstPtr& msgChatter){
	ROS_INFO("I heard chatter: [%.3f]", msgChatter->data);

	/*in = msg->data.c_str();

	while(*in) {
		printf("%f\n", strtod(in, &str));
		printf("Remainder: %s\n" ,str);
		in = str;*/
		/* move past the non-digits */
		/*while(!isdigit(*in) && *in) in++;
	}*/

}

void blatherCallback(const std_msgs::Float32::ConstPtr& msgBlather){
	ROS_INFO("I heard blather: [%.3f]", msgBlather->data);

	/*in = msg->data.c_str();

	while(*in) {
		printf("%f\n", strtod(in, &str));
		printf("Remainder: %s\n" ,str);
		in = str;*/
		/* move past the non-digits */
		/*while(!isdigit(*in) && *in) in++;
	}*/

}


int main(int argc, char **argv){

	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	ros::Subscriber sub1 = n.subscribe("chatter", 1000, chatterCallback);
	ros::Subscriber sub2 = n.subscribe("blather", 1000, blatherCallback);

	ros::spin();

	return 0;
}

