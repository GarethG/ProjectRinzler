#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "compassDriver.h"



int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "compass");	//inits the driver
	ros::NodeHandle n;			//this is what ROS uses to connect to a node

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("output", 1000);	/*advertise publishes a message, informs of the type of data
											and what the data is known by (i.e. its name). From here we
											also define the size of the buffer, so if we are producing
											data faster than we can send a large buffer is needed, else
											a smaller buffer should suffice */
	rosConfig(); //setup the ros stuff

	ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	int count = 0;
	while (ros::ok()){
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}


	return 0;
}
