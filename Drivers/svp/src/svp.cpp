#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEFAULT_BAUDRATE 19200
#define DEFAULT_SERIALPORT "/dev/ttyUSB0"

//Global data
FILE *fpSerial = NULL;   //serial port file pointer
ros::Publisher ucResponseMsg;
ros::Subscriber ucCommandMsg;
int ucIndex;          //ucontroller index number


//Initialize serial port, return file descriptor
FILE *serialInit(char * port, int baud){

	int BAUD = 0;
	int fd = -1;

	struct termios newtio;
	FILE *fp = NULL;

	//Open the serial port as a file descriptor for low level configuration
	// read/write, not controlling terminal for process,
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY );
	if ( fd<0 ){
		ROS_ERROR("serialInit: Could not open serial device %s",port);
		return fp;
	}

	// set up new settings
	memset(&newtio, 0,sizeof(newtio));
	newtio.c_cflag =  CS8 | CLOCAL | CREAD;  //no parity, 1 stop bit

	newtio.c_iflag = IGNCR;    //ignore CR, other options off
	newtio.c_iflag |= IGNBRK;  //ignore break condition

	newtio.c_oflag = 0;        //all options off

	newtio.c_lflag = ICANON;     //process input as lines

	// activate new settings
	tcflush(fd, TCIFLUSH);
	//Look up appropriate baud rate constant
	switch (baud){	
		case 38400:	default:	BAUD = B19200;	break;
		case 19200:	BAUD  = B19200;	break;
		case 9600:	BAUD  = B9600;	break;
		case 4800:	BAUD  = B4800;	break;
		case 2400:	BAUD  = B2400;	break;
		case 1800:	BAUD  = B1800;	break;
		case 1200:	BAUD  = B1200;	break;
	}  //end of switch baud_rate

	if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0){
		ROS_ERROR("serialInit: Failed to set serial baud rate: %d", baud);
		close(fd);
		return NULL;
	}

	tcsetattr(fd, TCSANOW, &newtio);
	tcflush(fd, TCIOFLUSH);

	//Open file as a standard I/O stream
	fp = fdopen(fd, "r+");
	if (!fp) {
		ROS_ERROR("serialInit: Failed to open serial stream %s", port);
		fp = NULL;
	}
	return fp;
} //serialInit


//Process ROS command message, send to uController
void ucCommandCallback(const std_msgs::String::ConstPtr& msg){
	ROS_DEBUG("uc%dCommand: %s", ucIndex, msg->data.c_str());
	fprintf(fpSerial, "%s", msg->data.c_str()); //appends newline
} //ucCommandCallback


//Receive command responses from robot uController
//and publish as a ROS message
void *rcvThread(void *arg){
	int rcvBufSize = 200;
	char ucResponse[rcvBufSize];   //response string from uController
	char *bufPos;


//	char buffer[MAX_BUFFER+1];
//	char p_buff[MAX_BUFFER+1];
//	char c[MAX_BUFFER+1];
//	int n,i,fi_col;

//	char * tok_ptr; //this is for splitting the serial stream into 2

	std_msgs::String msg;
	std::stringstream ss;

	ROS_INFO("rcvThread: receive thread running");

	while (ros::ok()) {
		bufPos = fgets(ucResponse, rcvBufSize, fpSerial);
		if (bufPos != NULL) {
			ROS_DEBUG("uc%dResponse: %s", ucIndex, ucResponse);
			msg.data = ucResponse;
			ucResponseMsg.publish(msg);
			printf("%s \n",bufPos); //print the buffer to the screen - should remove this when everything is working
			
			//String manipulation			
//			tok_ptr = strtok(buffer,"$");

//			strcpy(buffer,tok_ptr);
	
//			tok_ptr = strtok (buffer," ");
//			strcpy(p_buff,"");
//			fi_col = 1;
//			while (tok_ptr != NULL){
//				if(fi_col){
//					fi_col = 0;
//				}
//				else{
//					strcat(p_buff,"  ");
//				}
//				strcat(p_buff,tok_ptr);
//				tok_ptr = strtok (NULL," ");
//			}

			
		}
	}
	return NULL;
} //rcvThread


int main(int argc, char **argv){
	char port[20];    //port name
	int baud;     //baud rate 

	char topicSubscribe[20];
	char topicPublish[20];

	pthread_t rcvThrID;   //receive thread ID
	int err;

	//Initialize ROS
	ros::init(argc, argv, "r2SerialDriver");
	ros::NodeHandle rosNode;
	ROS_INFO("r2Serial starting");

	//Open and initialize the serial port to the uController
	if (argc > 1) {
		if(sscanf(argv[1],"%d", &ucIndex)==1) {
			sprintf(topicSubscribe, "uc%dCommand",ucIndex);
			sprintf(topicPublish, "uc%dResponse",ucIndex);
		}
		else {
			ROS_ERROR("ucontroller index parameter invalid");
			return 1;
		}
	}
	else {
		strcpy(topicSubscribe, "uc0Command");
		strcpy(topicPublish, "uc0Response");
	}

	//this is taking the argv argc stuff to configure the baud rate, we may implemtn this later but for now.
	strcpy(port, DEFAULT_SERIALPORT); //copy the #define for serial port and assign it to the port
	if (argc > 2){
		strcpy(port, argv[2]);
		baud = DEFAULT_BAUDRATE; //assign the 
	}
	if (argc > 3) {
		if(sscanf(argv[3],"%d", &baud)!=1) {
			ROS_ERROR("SVP baud rate parameter invalid");
			return 1;
		}
	}

	//for now force the default baud and port.
	strcpy(port, DEFAULT_SERIALPORT); //copy the #define for serial port and assign it to the port
	baud = DEFAULT_BAUDRATE; //assign the 

	ROS_INFO("connection initializing (%s) at %d baud", port, baud);
	fpSerial = serialInit(port, baud);
	if (!fpSerial ){
		ROS_ERROR("unable to create a new serial port");
		return 1;
	}
  
	ROS_INFO("serial connection successful");

	//Subscribe to ROS messages
	ucCommandMsg = rosNode.subscribe(topicSubscribe, 100, ucCommandCallback);

	//Setup to publish ROS messages
	ucResponseMsg = rosNode.advertise<std_msgs::String>(topicPublish, 100);

	//Create receive thread
	err = pthread_create(&rcvThrID, NULL, rcvThread, NULL);
	if (err != 0) {
		ROS_ERROR("unable to create receive thread");
		return 1;
	}

	//Process ROS messages and send serial commands to uController
	ros::spin();

	fclose(fpSerial);
	ROS_INFO("r2Serial stopping");
	return 0;
}

