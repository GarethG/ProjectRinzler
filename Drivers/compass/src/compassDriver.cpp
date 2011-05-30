#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */



#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "compassDriver.h"

int fd;

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "compass");	//inits the driver
	ros::NodeHandle n;			//this is what ROS uses to connect to a node

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("output", 1000);	/*advertise publishes a message, informs of the type of data
											and what the data is known by (i.e. its name). From here we
											also define the size of the buffer, so if we are producing
											data faster than we can send a large buffer is needed, else
											a smaller buffer should suffice */

	ros::Rate loop_rate(1); //how many times a second (i.e. Hz) the code should run

	if(!open_port()){
		printf("Port Open Failure\n");
		return 0;
	}
	
	if(!start_pni()){
		printf("Port Write Failed\n");
		return 0;
	}

	//int count = 0;
	while (ros::ok()){
		/*std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;*/

		fcntl(fd, F_SETFL, 0);

		printf("Read: %d\n",read);

	}


	return 0;
}

int open_port(void){

	fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1){
		perror("open_port: Unable to open /dev/ttyS0 - ");
	}

	return (fd);
}

int start_pni(void){
	return write(fd, "go", 2);
}
