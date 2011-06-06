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

#define DEFAULT_BAUDRATE 38400
#define DEFAULT_SERIALPORT "/dev/ttyS0"

#define kBufferSize	512
#define kPacketMinSize	5 
#define kDataCount	4
#define kHeading	5
#define kPAngle		24
#define	kRAngle		25
#define kTemperature	7
#define kSetDataComponents 3

using namespace std;

//Global data
FILE *fpSerial = NULL;   //serial port file pointer
ros::Publisher ucResponseMsg;
ros::Subscriber ucCommandMsg;
int ucIndex;          //ucontroller index number
int fd = -1;
char mOutData[kBufferSize];

unsigned int CRC(char * pData, unsigned int numBytes){
    unsigned int index = 0;
    unsigned int crc = 0;

    while(index < numBytes){
        crc = (unsigned int)(crc >> 8) | (crc << 8);
        crc ^= pData[index++];
        crc ^= (unsigned int)(crc & 0xff) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xff) << 4) << 1;
    }
    
    return crc;
}

unsigned int sendData(unsigned int frameType, void * dataPtr, unsigned int len){


	char * data = (char *)dataPtr;
	unsigned int index = 0;

	unsigned int crc;

	unsigned int count;


	count = (unsigned int)len + kPacketMinSize;

	if(len > kBufferSize - kPacketMinSize) return 0;

	mOutData[index++] = count >> 8;
	mOutData[index++] = count & 0xFF;
		
	mOutData[index++] = frameType ;
	
	while(len--) mOutData[index++] = *data++;

	crc = CRC(mOutData, index);
	mOutData[index++] = crc >> 8 ;
	mOutData[index++] = crc & 0xFF ;

	/*printf("index: %d\n",index);
	printf("%s\n",mOutData);
	for(index=0;index<10;index++){
		printf("%d %c\n",index,mOutData[index]);
	}*/

	return index;
}



//Initialize serial port, return file descriptor
FILE *serialInit(char * port, int baud){

	int BAUD = 0;

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
		case 38400:	default:	BAUD = B38400;	break;
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


void serialWrite(void){
	unsigned char buffer[]={0x00,0x05,0x04,0xbf,0x71};
	
	int wr;
	//unsigned int sz;


	/*unsigned char pkt[kDataCount + 1];

	pkt[0] = kDataCount;
	pkt[1] = kHeading;
	pkt[2] = kPAngle;
	pkt[3] = kRAngle;
	pkt[4] = kTemperature;

	sz = sendData(kSetDataComponents,pkt,kDataCount+1);

	wr=write(fd,mOutData,sz);*/

	wr=write(fd,buffer,sizeof(buffer));

	printf("Write returned: %d\n",wr);

	return;
}

unsigned char hex2bcd (unsigned char x){
    unsigned char y;
    y = (x / 10) << 4;
    y = y | (x % 10);
    return (y);
}


//Receive command responses from robot uController
//and publish as a ROS message
void *rcvThread(void *arg){
	int rcvBufSize = 500,i;
	char response[rcvBufSize];   //response string from uController
	unsigned char pBuff[rcvBufSize];
	char *bufPos;
	size_t bytes_read;
	std_msgs::String msg;
	std::stringstream ss;
	ros::Rate loop_rate(1); 

	ROS_INFO("rcvThread: receive thread running");

	while (ros::ok()) {
		for(i=0;i<rcvBufSize;i++){
			response[i] = 0;
		}

		serialWrite();
		bufPos = fgets(response, rcvBufSize, fpSerial);
		//bytes_read = fread(response,sizeof(response),1,fpSerial);
		if (bufPos != NULL) {
			/*ROS_DEBUG("uc%dResponse: %s", ucIndex, ucResponse);
			msg.data = ucResponse;
			ucResponseMsg.publish(msg);*/
			printf("I read %s\n",response);
			cout << response << endl;
			/*for(i=0;i<rcvBufSize;i++){
				printf("%c",response[i]);
			}*/
			//printf("\nBuffer size: %zu\n",bytes_read);
			for(i=0;i<100;i++){
				pBuff[i] = hex2bcd(response[i]);
				printf("%d: %cB ",i,pBuff[i]);
			} 
				
			/*for(i=0;i<rcvBufSize;i++){
				printf("%d: %c\n",i,response[i]);
			}*/
		}
		else{
			printf("no data\n");
		}
		loop_rate.sleep();
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

	strcpy(port, DEFAULT_SERIALPORT);
	if (argc > 2){
		strcpy(port, argv[2]);
		baud = DEFAULT_BAUDRATE;
	}
	if (argc > 3) {
		if(sscanf(argv[3],"%d", &baud)!=1) {
			ROS_ERROR("ucontroller baud rate parameter invalid");
			return 1;
		}
	}

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
