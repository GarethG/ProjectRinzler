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

#include <sstream>
#include "compassDriver.h"

int fd; 				/* File descriptor for the port */
unsigned char returnBuffer[10000]; 	/*Buffer which stores read data*/
unsigned char *rBptr;			/*Ptr*/

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
		return 0;	//we failed to open the port so end
	}

	config_port();

	while (ros::ok()){

		if(write_port()){	//if we send correctly
			if(read_port()){	//if we read correctly
				parseBuffer();	//parse the buffer
			}
			else{
				ROS_ERROR("Read no data");
			}
		}
		else{
			ROS_ERROR("Failed to write");
		}

		loop_rate.sleep();

	}


	return 0;
}

/*********************************
** Opens serial port S0		**
*********************************/

int open_port(void){	

	fd = open("/dev/ttyS0", O_RDWR | O_NDELAY );//| O_NOCTTY);
	if (fd == -1){
		ROS_ERROR("Could not open port");
		return 0;
	}
	else{
		fcntl(fd, F_SETFL, 0);
		ROS_INFO("Port opened with a descritor of %d",fd);
	}
	return (fd);
}

/*********************************
** Configures the serial port	**
*********************************/

void config_port(void){
	struct termios options;

	tcgetattr(fd, &options);

	cfsetispeed(&options, B38400);
	cfsetospeed(&options, B38400);

	options.c_cflag |= (CLOCAL | CREAD);

	tcsetattr(fd, TCSANOW, &options);

	return;
}

/*************************************************
** Tries to transmit the message to the compass	**
** which will get it to send some info back	**
*************************************************/

int write_port(void){
	int n;
	char buffer[]={0x00,0x05,0x04,0xbf,0x71};

	n = write(fd,buffer,sizeof(buffer));

	if (n < 0){
		ROS_ERROR("Failed to write to port");
		return 0;
	}
	return (n);
}

/*************************************************
** Reads data off of the serial port and will	**
** then return the data into returnBuffer	**
*************************************************/

int read_port(void){	
	return read(fd,returnBuffer,sizeof(returnBuffer));
}

/*************************************************
** When called will sift through the buffer in	**
** an attempt to obtain a uint 32		**
*************************************************/

unsigned int getU32(void){

	unsigned int tmp1, tmp2, tmp3, tmp4;

	tmp1 = *rBptr++;	//get the first item
	tmp2 = *rBptr++;	//get the second item
	tmp3 = *rBptr++;	//get the third item
	tmp4 = *rBptr++;	//get the fourth item

	tmp1 <<= 24;
	tmp2 <<= 16;
	tmp3 <<= 8;

	tmp1 = tmp1 | tmp2 | tmp3 | tmp4;

	return tmp1;
}

/*************************************************
** When called will sift through the buffer in	**
** an attempt to obtain a uint 16		**
*************************************************/

unsigned int getU16(void){

	unsigned int tmp1, tmp2;

	tmp1 = *rBptr++;	//get the first item
	tmp2 = *rBptr++;	//get the second item

	tmp1 <<= 8;		//shift 8 bits left

	tmp1 |= tmp2;		//or to combine data

	return tmp1;		//return the U16
}

/*************************************************
** When called will sift through the buffer in	**
** an attempt to obtain a uint 8		**
*************************************************/

unsigned int getU8(void){
	return *rBptr++;	//returns the current point on the array and shifts along (a U8)
}

/*************************************************
** When called will sift through the buffer in	**
** an attempt to obtain a float			**
*************************************************/

float getF32(void){
	unsigned int tmp1;

	tmp1 = getU32();
	
	return *((float*)&tmp1);
}

/*Parses buffer*/
	

void parseBuffer(void){
	
	unsigned int parsed = 0;
	float parsedF = 0.0;

	rBptr = &returnBuffer[0];	//assign the start of the char buffer

	parsed = getU16();		//get number of bytes transmited

	//printf("bytes recieved: %u\n",parsed);

	parsed = getU8();		//get frame ID

	//printf("frame ID: %u\n",parsed);

	parsed = getU8();		//get ID count

	//printf("ID count: %u\n",parsed);

	parsed = getU8();		//get ID

	//printf("first ID: %u\n",parsed);

	parsedF = getF32();		//get compass

	printf("Compass: %f ",parsedF);

	parsed = getU8();		//get ID

	//printf("second ID: %u\n",parsed);

	parsedF = getF32();		//get pitch

	printf("Pitch: %f ",parsedF);

	parsed = getU8();		//get ID

	//printf("third ID: %u\n",parsed);

	parsedF = getF32();		//get pitch

	printf("Roll: %f",parsedF);

	printf("\n");

	return;
}


