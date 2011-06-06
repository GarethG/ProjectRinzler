#include <stdio.h>

#include <string.h>

#include <fcntl.h>

#include <errno.h>

#include <termios.h>

#include <unistd.h>
/*
#include "ros/ros.h"
#include "std_msgs/String.h"
*/

int fd1;

char *bufptr;

int wr,rd;


int main(void){//int argc, char **argv){

	unsigned char buffer[]={0x00,0x05,0x04,0xbf,0x71};

	char buff[10];

	int i;
	int BAUD = 0;

	struct termios newtio;

	/*ros::init(argc, argv, "r2SerialDriver");
	ros::NodeHandle rosNode;
	ROS_INFO("r2Serial starting");
	std_msgs::String msg;
	std::stringstream ss;
	ros::Rate loop_rate(1); */


	fd1=open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);

	if(fd1 == -1 ){
		perror("open_port: Unable to open /dev/ttyS0 – ");

	}
	else{
		fcntl(fd1, F_SETFL,0);
		printf("Port 1 has been sucessfully opened and %d is the file description\n",fd1);
	}

	// set up new settings
	memset(&newtio, 0,sizeof(newtio));
	newtio.c_cflag =  CS8 | CLOCAL | CREAD;  //no parity, 1 stop bit

	newtio.c_iflag = IGNCR;    //ignore CR, other options off
	newtio.c_iflag |= IGNBRK;  //ignore break condition

	newtio.c_oflag = 0;        //all options off

	newtio.c_lflag = ICANON;     //process input as lines

	// activate new settings
	tcflush(fd1, TCIFLUSH);
	BAUD = B38400;
	

	if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0){
		//ROS_ERROR("serialInit: Failed to set serial baud rate: %d", baud);
		printf("lol fail\n");
		close(fd1);
		return NULL;
	}

	tcsetattr(fd1, TCSANOW, &newtio);
	tcflush(fd1, TCIOFLUSH);

	/*close(fd1);

	return 0;*/
	
	while (1){//ros::ok()) {

		printf("Attempting to write %s\n",buffer);	

		wr=write(fd1,buffer,sizeof(buffer));

		printf("Attempting to read buffer\n");

		rd=read(fd1,buff,10);

		printf("I read: %s\n",buff);

		for(i=0;i<sizeof(buff);i++){
			printf("%c\n",buff[i]);
		}

		printf("end loop\n");

		sleep(1);

		//loop_rate.sleep();
	}

	//printf("Bytes sent are %d \n",rd);

	close(fd1);

	return 0;

}
