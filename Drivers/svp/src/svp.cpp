//testing to
//next to thrusters and next to aluminium pole


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
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
#include <iostream>
#include "svp.h"
//#include <math.h>

using namespace std;


#define DEFAULT_BAUDRATE 19200
#define DEFAULT_SERIALPORT "/dev/ttyUSB0"

//Global data
FILE *fpSerial = NULL;   //serial port file pointer
ros::Publisher svpResponseMsg;
ros::Publisher svpDepthMsg;
ros::Publisher svpVeloMsg;
//ros::Subscriber svpCommandMsg;



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

//**********Transmit to SVP - Not currently implemented
//Process ROS command message, send to uController
//void ucCommandCallback(const std_msgs::String::ConstPtr& msg){
//	ROS_DEBUG("uc%dCommand: %s", ucIndex, msg->data.c_str());
//	fprintf(fpSerial, "%s", msg->data.c_str()); //appends newline
//} //ucCommandCallback

//Receive data from the SVP 
//and publish as a ROS message
void *rcvThread(void *arg){
	int rcvBufSize = 200;
	char svpResponse[rcvBufSize];   //response string from uController
	double svpDepth;
	char *bufPos;
	
	char depth[10] = "0000000", *dEnd;
	float depthFloat;
	float oldDepth;
	float pressure;

	char velo[10] = "0000000", *vEnd;
	float veloFloat;
	float oldVelo;
	int i;



	//std_msgs::String depth;
	std_msgs::String msg;
	std_msgs::Float64 dMsg;
	std_msgs::Float64 vMsg;
	std::stringstream ss;

	ROS_INFO("rcvThread: receive thread running on SVP node");

	while (ros::ok()) {
		bufPos = fgets(svpResponse, rcvBufSize, fpSerial);
		if (bufPos != NULL) {
			ROS_DEBUG("svpResponse: %s", svpResponse);
			msg.data = svpResponse;
			//printf("%s\n",bufPos);
			//split svpResponse

			//The Depth			
			for (i = 0; i < 6; i++){
				depth[i] = bufPos[i+1]; 
			}
			double depthFloat = strtod(depth, &dEnd);
			//heres a little idea on how to handle NaN(Not a Number) cases			
			//if (isnan(depthFloat) != NULL){
			//	printf("ERROR wierdness happened\n");
			//}

			depthFloat -= SURFACE;	//pressure - surface pressure = specific weight x depth
			depthFloat /= DENSITY;
			
			//if depthfloat != to a float...
			dMsg.data = depthFloat;	//dMsg = 1.01240;//
			//printf("depth - %lf\n", depthFloat);
			

			//The Velocity
			for (i = 1; i < 10; i++){
				
				velo[i] = bufPos[i+7]; 
				
			}
			//printf("raw velo %s\n",velo);
			double veloFloat = strtod(velo, &vEnd);
			vMsg.data = veloFloat;
			//printf("Velocity - %lf\n", veloFloat);

			
			svpResponseMsg.publish(msg); //the whole message
			svpDepthMsg.publish(dMsg); //the whole message 
			svpVeloMsg.publish(vMsg); //the whole message 
			//printf("%s \n",bufPos); //print the buffer to the screen - should remove this when everything is working
			
					
		}
	}
	return NULL;
} //rcvThread


int main(int argc, char **argv){
	char port[20];    //port name
	int baud;     //baud rate 

	char topicSubscribe[20];
	char topicPublish[20];
	char topicPubDepth[20];
	char topicPubVelo[20];
	pthread_t rcvThrID;   //receive thread ID
	int err;

	//Initialize ROS
	ros::init(argc, argv, "svpDriver");
	ros::NodeHandle rosNode;
	ROS_INFO("SVP starting");

	//Open and initialize the serial port to the uController
	//if (argc > 1) {
	//	if(sscanf(argv[1],"%d", &svpIndex)==1) {
	//		//sprintf(topicSubscribe, "svp%dCommand",ucIndex);
	//		sprintf(topicPublish, "svp%dResponse",svpIndex);
	//	}
	//	else {
	//		ROS_ERROR("SVP index parameter invalid");
	//		return 1;
	//	}
	//}
	//else {
		//strcpy(topicSubscribe, "svpCommand");
		strcpy(topicPublish, "svpRawResponse");
		strcpy(topicPubDepth, "svpDepth");
		strcpy(topicPubVelo, "svpVelocity");
	//}

	//this is taking the argv argc stuff to configure the baud rate, we may implement this later but for now.
	/*strcpy(port, DEFAULT_SERIALPORT); //copy the #define for serial port and assign it to the port
	if (argc > 2){
		strcpy(port, argv[2]);
		baud = DEFAULT_BAUDRATE; //assign the 
	}
	if (argc > 3) {
		if(sscanf(argv[3],"%d", &baud)!=1) {
			ROS_ERROR("SVP baud rate parameter invalid");
			return 1;
		}
	}*/

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
	//svpCommandMsg = rosNode.subscribe(topicSubscribe, 100, svpCommandCallback);

	//Setup to publish ROS messages
	svpResponseMsg = rosNode.advertise<std_msgs::String>(topicPublish, 100); //This is the raw serial data, is this worth keeping?
	svpDepthMsg = rosNode.advertise<std_msgs::Float64>(topicPubDepth, 100); 
	svpVeloMsg = rosNode.advertise<std_msgs::Float64>(topicPubVelo, 100);
	//Create receive thread
	err = pthread_create(&rcvThrID, NULL, rcvThread, NULL);
	if (err != 0) {
		ROS_ERROR("unable to create receive thread");
		return 1;
	}

	//Process ROS messages
	ros::spin();

	fclose(fpSerial);
	ROS_INFO("SVP stopping");
	return 0;
}

