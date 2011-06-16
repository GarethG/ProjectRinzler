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
#include "compassDriver.h"

int fd; 				/* File descriptor for the port */
unsigned char returnBuffer[10000]; 	/*Buffer which stores read data*/
unsigned char *rBptr;			/*Ptr*/
float heading, pitch, roll;		/*Floats for the returned values*/

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "compass");	//inits the driver
	ros::NodeHandle n;			//this is what ROS uses to connect to a node

	/*Advertises our various messages*/

	ros::Publisher compassHeadingMsg = n.advertise<std_msgs::Float32>("compassHeading", 100);
	ros::Publisher compassPitchMsg = n.advertise<std_msgs::Float32>("compassPitch", 100);
	ros::Publisher compassRollMsg = n.advertise<std_msgs::Float32>("compassRoll", 100);

	/*Sets up the message structures*/

	std_msgs::Float32 compassHeading;
	std_msgs::Float32 compassPitch;
	std_msgs::Float32 compassRoll;

	ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	if(!open_port()){
		return 0;	//we failed to open the port so end
	}

	config_port();

	ROS_INFO("Compass Driver Online");

	while (ros::ok()){

		if(write_port()){	//if we send correctly
			if(read_port()){	//if we read correctly
				
				parseBuffer();	//parse the buffer
				//printf("H: %f P: %f R: %f\n",heading,pitch,roll);

				/* Below here sets up the messages ready for transmission*/

				compassHeading.data = heading;	
				compassPitch.data = pitch;
				compassRoll.data = roll;
				
			}
			else{
				ROS_ERROR("Read no data");
			}
		}
		else{
			ROS_ERROR("Failed to write");
		}

		/*Below here we publish our readings*/

		compassHeadingMsg.publish(compassHeading);
		compassRollMsg.publish(compassRoll);
		compassPitchMsg.publish(compassPitch);

		/*Have a snooze*/

		loop_rate.sleep();

	}

	ros::spin();

	close(fd);
	ROS_INFO("Shutting Down");
	printf("Shutting Down\n");

	return 0;
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

	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	tcflush(fd, TCIFLUSH);
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
	//char buffer[]={0x00,0x05,0x01,0xef,0xd4};

	n = write(fd,buffer,sizeof(buffer));

	if (n < 0){
		ROS_ERROR("Failed to write to port");
		return 0;
	}
	/*else{
		printf("We Transmitted %d\n",n);
	}*/
	return (n);
}

/*************************************************
** Reads data off of the serial port and will	**
** then return the data into returnBuffer	**
*************************************************/

int read_port(void){	

	int n;
	//int i;

	n = read(fd,returnBuffer,sizeof(returnBuffer));

	/*printf("We read %d bytes\n",n);


	for(i=0;i<=n;i++){
		if(i!=0){
			printf(":");
		}
		printf("%x",returnBuffer[i]);
	}

	printf("\n\n");	*/

	return n;
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

/*************************************************
** Runs through the read data and pulls out all	**
** of the important information, specifically	**
** the heading pitch and roll. Items such as 	**
** bytes transmited frame IDs etc are all 	**
** stored for debugging purposes. However due	**
** to the design they must be called else we	**
** will have to shuffle through the ptrs using	**
** guess work (or decision work) to get the ptr	**
** to the correct location to pull the floats	**
** out.						** 
*************************************************/	

void parseBuffer(void){
	
	unsigned int parsed[6];

	rBptr = &returnBuffer[0];	//assign the start of the char buffer

	parsed[0] = getU16();		//get number of bytes transmited
	parsed[1] = getU8();		//get frame ID
	parsed[2] = getU8();		//get ID count
	parsed[3] = getU8();		//get ID
	heading = getF32();		//get compass heading
	parsed[4] = getU8();		//get ID
	pitch = getF32();		//get pitch
	parsed[5] = getU8();		//get ID
	roll = getF32();		//get pitch

	return;
}


