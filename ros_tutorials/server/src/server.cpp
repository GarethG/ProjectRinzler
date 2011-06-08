#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>

int main(int argc, char **argv){

	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("chatter", 1000);
	ros::Publisher blather_pub = n.advertise<std_msgs::Float32>("blather", 1000);

	ros::Rate loop_rate(1);

	int count = 0;
	while (ros::ok()){

		//std_msgs::String msg;
		std_msgs::Float32 msgChatter;
		std_msgs::Float32 msgBlather;

		/*std::stringstream ss;
		ss << "hello world " << ":" << count << ":" << value;
		msg.data = ss.str();*/



		if(count == 10){

			msgBlather.data = 654.321;
			ROS_INFO("Blather: %.3f", msgBlather.data);
			blather_pub.publish(msgBlather);
			count = 0;
		}
		else{
			msgChatter.data = 123.456;
			ROS_INFO("Chatter: %.3f", msgChatter.data);
			chatter_pub.publish(msgChatter);
		}

		ros::spinOnce();

		loop_rate.sleep();

		++count;
	}


	return 0;
}
