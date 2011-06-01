#include <stdio.h>

#include <string.h>

#include <fcntl.h>

#include <errno.h>

#include <termios.h>

#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"


int fd1;

int fd2;

char *buff,*buffer,*bufptr;

int wr,rd,nbytes,tries;


int main(int argc, char **argv){

	unsigned char buffer[]={0x00,0x05,0x04,0xbf,0x71};


	ros::init(argc, argv, "r2SerialDriver");
	ros::NodeHandle rosNode;
	ROS_INFO("r2Serial starting");



	fd1=open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);

	if(fd1 == -1 ){
		perror("open_port: Unable to open /dev/ttyS0 – ");

	}
	else{
		fcntl(fd1, F_SETFL,0);
		printf("Port 1 has been sucessfully opened and %d is the file description\n",fd1);
	}

	

	wr=write(fd1,buffer,sizeof(buffer));

	

	//rd=read(fd1,buff,10);

	//printf("Bytes sent are %d \n",rd);

	close(fd1);

	return 0;

}
