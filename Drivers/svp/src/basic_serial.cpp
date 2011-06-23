#include <stdio.h>   
#include <unistd.h>  
#include <fcntl.h>   
#include <errno.h>   
#include <termios.h> 
#include <stdlib.h>
#include <math.h>
#include <string.h>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>

int fd; 				/* File descriptor for the port */
unsigned char returnBuffer[100]; 	/*Buffer which stores read data*/
unsigned char *rBptr;			/*Ptr*/
float heading, pitch, roll;		/*Floats for the returned values*/

int read_port(void){	

	int n;
	int i;

	n = read(fd,returnBuffer,sizeof(returnBuffer));

	

	printf("We read %d bytes\n",n);


	for(i=0;i<=n;i++){
		if(i!=0){
			printf(":");
		}
		printf("%x",returnBuffer[i]);
	}

	printf("\n\n");

	return n;
}


/*********************************
** Opens serial port S0		**
*********************************/

int open_port(void){	

	fd = open("/dev/ttyS0", O_RDWR | O_NDELAY | O_NOCTTY);
	if (fd == -1){
		ROS_ERROR("Could not open port");
		return 0;
	}
	else{
		fcntl(fd, F_SETFL, 0);

		ROS_INFO("Port opened with a descriptor of %d",fd);
	}
	return (fd);
}

/*********************************
** Configures the serial port	**
*********************************/

void config_port(void){
	struct termios options;

	tcgetattr(fd, &options);

	cfsetispeed(&options, B19200);
	cfsetospeed(&options, B19200);

	options.c_cflag |= (CLOCAL | CREAD | CS8);

	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME] = 10;
	options.c_cc[VMIN] = 0;
	tcflush(fd, TCIFLUSH);

	tcsetattr(fd, TCSANOW, &options);

	return;
}


int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "svp");	//inits the driver
	ros::NodeHandle n;			//this is what ROS uses to connect to a node

	/*Advertises our various messages*/

	ros::Publisher svpMsg = n.advertise<std_msgs::Float32>("svp", 100);

	/*Sets up the message structures*/

	std_msgs::Float32 svp;

	ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	if(!open_port()){
		return 0;	//we failed to open the port so end
	}

	config_port();

	ROS_INFO("SVP Driver Online");

	while (ros::ok()){

		if(read_port() != 0){	//if we read correctly
			
			//parseBuffer();	//parse the buffer
			//printf("H: %f P: %f R: %f\n",heading,pitch,roll);

			/* Below here sets up the messages ready for transmission*/

			
		}
		else{
			ROS_ERROR("Read no data");
		}
	
		/*Below here we publish our readings*/

		/*Have a snooze*/

		loop_rate.sleep();

	}

	ros::spin();

	close(fd);
	ROS_INFO("Shutting Down");
	printf("Shutting Down\n");

	return 0;
}

