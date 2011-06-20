#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h> 
#include <fcntl.h> 
#include <sys/time.h>

#include "ros/ros.h"
#include "sonarDriver.h"

#define SENDBUFFSIZE 82

int fd; 							/* File descriptor for the port */
unsigned char returnBuffer[500]; 	/*Buffer which stores read data*/
unsigned char *rBptr;				/*Ptr*/

unsigned char sendBuffer[82];
	
unsigned char header,	//Message Header. 
			hLength,	//Hex Length of whole binary packet
			bLength,	//Binary Word of above Hex Length.
			sID,		//Packet Source Identification
			dID,		//Packet Destination Identification
			byteCount,	//Byte Count of attached message that follows this byte.
			msg[263],	//Command / Reply Message
			term;		//Message Terminator

unsigned int bp1_temp[263],	//Clone dataset, for bad packet recovery
			bp1_buffLen,
			bp1_bLength;

/******************************************************
 * 
 *  Main; the master of functions, the definer of variables.
 * 
 * ***************************************************/


int main( int argc, char **argv )
{

	int cmd = 0;
	int o;

/************ Open and Configure the Serial Port. ************/

	open_port();	
	config_port();
	
	while(cmd != 1)
	{
		
/************ Initilaise the sonar **************/
		if(initSonar() == 1)
		{
			
			while(cmd != mtAlive)
			{
				if(sortPacket() == 1)
				{
					cmd = returnMsg();
					//printf("%d : ", cmd);
	
					//Check for mtAlive
					if(cmd == mtAlive)
					{
						//printf("blablabla\n");
					
					}
				}
			}
			
			usleep(500);
			
			//Make and send headcommand
					
			// Range, Left Angle, Right Angle, ADSpam, ADLow, Gain, ADInterval, Number of Bins.
			makeHeadPacket(5, 0, 6399, 81, 8, 84, 141, 90);
			printf(">> mtHeadCommand\n");

				
				
			if(sortPacket() == 1)
			{
				
				cmd = returnMsg();
				//printf("%d : ", cmd);
				//Should return mtAlive 
				//(need to check params too but not implemented yet)
				if(cmd == mtAlive)
				{
					printf("\t<< mtAlive!\n");
					
					while(1)
					{
			
					
						//Make the sendData packet and send
						makePacket(mtSendData);
						printf(">> mtSendData\n");
						
						usleep(500);
						//if you get some data back go forth
						if(sortPacket() == 1)
						{
							
							cmd = returnMsg();
							//printf("hi %d\n", cmd);
							
							//If it's mtHeadData we winning.
							if(cmd == mtHeadData)
							{
								printf("\t<< mtHeadData!!\n");
								while(1)
								{
									//winning
								}
							}
							//end mtHeadData
							
						}
						//end sortPacet() == 1	
					}
					
				}
				//mtAlive check							
				

			}
			//end sortPacket() == 1				

		}
		
		
	}	
	close(fd);
	return 0;
}

/*********************************
** Opens serial port S0		**
*********************************/

int open_port(void){	

	fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY);// O_NDELAY |
	//printf("/dev/ttyS0\n");
	if (fd == -1){
		ROS_ERROR("Could not open port");
		return 0;
	}
	else{
		fcntl(fd, F_SETLK, 0);
		ROS_INFO("Port opened with a descriptor of %d",fd);
	}
	return (fd);
}

/*********************************
** Configures the serial port	**
*********************************/

void config_port(void){
	struct termios options1;

	tcgetattr(fd, &options1);

	options1.c_cflag |= (CLOCAL | CREAD);// | CS8
	options1.c_cflag &=~(CSIZE);
	options1.c_cflag |= CS8;
	options1.c_cflag &=~(PARENB);
	options1.c_cflag &=~(CSTOPB);
	
	cfsetispeed(&options1, BAUDRATE);
	cfsetospeed(&options1, BAUDRATE);

	//options1.c_iflag = IGNPAR;//no parrot-tea
	options1.c_lflag &=~(ICANON | ECHO | ECHOE | ISIG);
	options1.c_oflag = ~OPOST;//1;
	options1.c_iflag &= ~(IXON | IXOFF | IXANY);
	options1.c_cc[VTIME] = 10;
	options1.c_cc[VMIN] =0;
	tcflush(fd, TCIFLUSH);

	tcsetattr(fd, TCSANOW, &options1);

	return;
}

/*************************************************
** Tries to transmit the message to the compass	**
** which will get it to send some info back	**
*************************************************/

int write_port(unsigned char sendBuffer[SENDBUFFSIZE], unsigned int sendSize){
	
	int n;

	n = write(fd,sendBuffer, sendSize);

	if (n < 0){
		ROS_ERROR("Failed to write to port");
		return 0;
	}
	else{
		//printf("We Transmitted %d\n",n);
	}
	sleep(1);
	return (n);
}

/*************************************************
** Reads data off of the serial port and will	**
** then return the data into returnBuffer	**
*************************************************/

int read_port(void){	

	int n;

	n = read(fd, returnBuffer,sizeof(returnBuffer));

	//printf("We read %d bytes\n", n);

	//printf("\n\n");
	
	//	printf("<< ");
	//	for(j = 0; j < n; j ++)
	//		printf("%x : ", returnBuffer[j]);
	//	printf("\n");	

	rBptr = &returnBuffer[0];
	//sleep(1);
	return n;
}

/*************************************************
** When called will add together the inputs to make a U32
* hacked from taylords code
*************************************************/

unsigned int getU32(unsigned int tmp1, unsigned int tmp2, unsigned int tmp3, unsigned int tmp4)
{

	tmp1 <<= 24;
	tmp2 <<= 16;
	tmp3 <<= 8;

	tmp1 = tmp1 | tmp2 | tmp3 | tmp4;

	return tmp1;
}

/*************************************************
** When called will add together the inputs to make a U16
* hacked from taylords code
*************************************************/

unsigned int getU16(unsigned int tmp1, unsigned int tmp2)
{

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

/**************************************************************** 
SeaNet General Packet Format is;
*	'@'		:	Message Header = '@' (0x40).
*	HHHH	:	Hex Length of whole binary packet (excluding LF Terminator).
*	BB		:	Binary Word of above Hex Length.
*	SID		:	Packet Source Identification (Tx Node number 0 - 255).
*	DID		:	Packet Destination Identification (Rx Node number 0 -255).
*	COUNT	:	Byte Count of attached message that follows this byte.
*	MSG		:	Command / Reply Message (i.e. 'Alive' command, 'Data Reply' message).
*	TERM	:	Message Terminator = Line Feed (0Ah).
* 
*	Returns -1 if failed, 1 if correct first time, 2 if it had to 
* 	stich packets
* 
****************************************************************/
int sortPacket(void)
{

	int packetFlag = 0,
	leFlag = 0,
	buffLen, 		//length of the recieved buffer, output from read_port()
	i,				//counter
	temp[263],
	msgLen;
	
	//How long was the msg, according to read() ?
	buffLen = read_port();
	//printf("buffLen = %d\n", buffLen);
	rBptr = &returnBuffer[0];	//set pointer for recieved data
			
	//Store all packets into temp[]	
	
	for( i = 0; i < buffLen; i++ )
	{
		leFlag = temp[i];
		temp[i] = getU8();
		if(i >= 1 && temp[i] == 0x40 && temp[i-1] == 0x0a)
		{
			buffLen = i;
			printf("buffLen %d - %x\n", buffLen, temp[buffLen]);
		}
		
	}

	//Store temp stuff into right place
	
	header = temp[0];
	hLength = getU32(temp[1], temp[2], temp[3], temp[4]);
	bLength = getU16(temp[6], temp[5]);
	sID = temp[7];
	dID = temp[8];
	byteCount = temp[9];
	term = temp[buffLen-1];
	
	//Clear msg buffer first
	for(i = 0; i < 263; i++)
		msg[i] = NULL;
		
	msgLen = 0;
	
	//Store the msg, works for varying lengths
	for( i = 10; i < (buffLen-2); i ++)
	{
		msg[(i-10)] = temp[i];
		msgLen ++;
	}

	
	//What is prinf?

	if(header == '@' && term == 10 && buffLen == bLength + 6)
	{

		printf("<< ");
		for(i = 0; i < buffLen; i ++)
			printf("%x : ", temp[i]);
		printf("\n");
		
		packetFlag = 1;
		
		
		if(msg[0] == mtHeadData)
		{
			
			printf("WINNING\n");
			for(i = 44; i < buffLen-1; i++ )
				printf("%d, ", temp[i]);
			printf("\n");
			
		}
		
		//printf("%d, %d\n", byteCount, msgLen+1);
	}
	else
	{
		packetFlag = -1;
	}
	
	return packetFlag; 
}

/*************************************************
 * just returns the first value of the msg 
 * *********************************************/
int returnMsg()
{
	return msg[0];
}

/************************************************
 * 
 *  Create a packet to send and stores it in sendBuffer
 * takes the command and returns the command that has
 * been sent.
 * 
 * *********************************************/
void makePacket(int command)
{
	
	unsigned int j = 0;
	
	/************ 14 byte commands ***********************/
	
	if(command == mtReBoot || command == mtSendVersion || command == mtSendBBUser)
	{
		sendBuffer = {	0x40, 		//Header
						0x30, 
						0x30, 
						0x30, 
						0x38, 
						0x08, 
						0x00, 
						0xFF, 
						0x02, 
						0x03, 
						command, 
						0x80, 
						0x02, 
						0x0A };		//Footer
						
		//Send		
		write_port(sendBuffer, 14);
		
	}
	
	/************ 18 byte commands ***********************/

	else if(command == mtSendData)
	{		
		sendBuffer = {	0x40, 		//Header
						0x30, 
						0x30, 
						0x30, 
						0x43, 
						0x0C, 
						0x00, 
						0xFF, 
						0x02, 
						0x07, 
						command, 
						0x80, 
						0x02, 
						0x00, //buff[0], 
						0x00, //buff[1], 
						0x00, //buff[2], 
						0x00, //buff[3], 
						0x0A };		//Footer
		
		//Send
		write_port(sendBuffer, 18);
		
		
	}
	else
	{
		printf("Error: Unknown Command.\n");
	}

	/********* Print the packet that's being sent **********/
	printf(">> ");
	while(sendBuffer[j] != 0x0a)
	{									
		printf("%x : ", sendBuffer[j]);
		j ++;
	}
	printf("%x", sendBuffer[j]);
	printf("\n");
	
}

/************************************************
 *  make a packet for sending mtHeadCommand
 * *********************************************/
void makeHeadPacket(unsigned int range, unsigned int startAngle, unsigned int endAngle, unsigned int ADspan, 
					unsigned int ADlow, unsigned int gain, unsigned int ADInterval, unsigned int numBins)
{
	
	int j = 0;
	int drange = range * 10;
	const unsigned int MAX_GAIN = 210;
	unsigned int gainByte = (gain * MAX_GAIN);
	
					//Step 1, setup
	sendBuffer = { 	0x40,						//Header
					0x30,						//Hex Length
					0x30,
					0x34,
					0x43,
					0x4C,						//Binary Length
					0x00,
					0xFF,						//Source ID
					0x02,						//Destination ID
					0x47,						//Byte Count
					//Step 2, Message
					0x13,						//Command
					0x80,
					0x02,
					0x1D,						//Head Command Type - 1 = Normal, 29 Dual Channel
					0x85,						//HdCtrl bytes
					0x23,
					0x03,						//Head Type
					0x99,						//TxN Channel 1
					0x99,
					0x99,
					0x02,
					0x66,						//TxN Channel 2
					0x66,
					0x66,
					0x05,
					0xA3,						//RxN Channel 1
					0x70,
					0x3D,
					0x06,
					0x70,						//RxN Channel 2
					0x3D,
					0x00,
					0x09,
					0x28,						//TxPulse Length
					0x00,
					(drange & 0xFF),			//Range Scale
					((drange >> 8) & 0xFF),
					(startAngle & 0xFF),		//Left Angle Limit
					((startAngle >> 8) & 0xFF),	
					(endAngle & 0xFF),			//Right Angle Limit
					((endAngle >> 8) & 0xFF),
					(ADspan * 255 / 80),
					(ADlow * 255 / 80),
					gainByte,					//IGain Settings	
					gainByte,
					0x5A,						//Slope Setting
					0x00,
					0x7D,
					0x00,
					0x20,						//MoTime
					16,							//Step Angle Size
					(ADInterval & 0xFF),		//ADInterval
					((ADInterval >> 8) & 0xFF),
					(numBins & 0xFF),			//Number of Bins
					((numBins >> 8) & 0xFF),
					0xE8,						//MaxADbuf
					0x03,
					0x97,						//Lockout
					0x03,
					0x40,						//Minor Axis Direction
					0x06,
					0x01,						//Major Axis Direction
					0x00,						//Ctrl2
					0x00,						//ScanZ
					0x00,
					//Step 3, ???
					0x50,
					0x51,
					0x09,
					0x08,
					0x54,
					0x54,
					0x00,
					0x00,
					0x5A,
					0x00,
					0x7D,
					0x00,
					0x00,
					0x00,
					0x00,
					0x00,
					//Step 4, Profit.
					0x0A};						//Terminator (2, 1 is pretty terrible.)


	/********* Print the packet that's being sent **********/
	printf(">> ");
	while(sendBuffer[j] != 0x0a)
	{									
		printf("%x : ", sendBuffer[j]);
		j ++;
	}
	printf("%x", sendBuffer[j]);
	printf("\n");
	
	write_port(sendBuffer, 82);
	
}

/**********************************************
 * Find the length of the packet
 * Flag = 0 - sendBuffer
 * Flag = 1 - recvBuffer
 * Returns packet length, -1 if error.
 * *******************************************/
int packetLength(int flag)
{
	
	int len, temp = 0;
	//rcvBuffer
	if(flag == 1)
	{
	
		while(1)
		{
			while( returnBuffer[len] != 0 )
			{
				len++;
				
				if( returnBuffer[len] == 0 && temp == 0x10 )
					return len;
				
				temp = returnBuffer[len];
			}
		}
		
	}
	else
		return -1;
	
	
}
/**********************************************
 * Initilise sonar,
 * check alive, send version request, check for
 * reply
 * returns 1 if set up, -1 if failed.
 * *******************************************/
int initSonar(void)
{
	int cmd;
	//Read Port
	if(sortPacket() == 1)
	{
		cmd = returnMsg();
		//printf("%d : ", cmd);
	
		//Check for mtAlive
		if(cmd == mtAlive)
		{
			printf("\t<< mtAlive!\n");
			//Make mtSendVersion packet and send
			makePacket(mtSendVersion);
			printf(">> mtSendVersion\n");
			
			//Read Port
			if(sortPacket() == 1)
			{
				
				cmd = returnMsg();
				
				//Check for mtVersionData
				if(cmd == mtVersionData)
				{
					printf("\t<< mtVersionData!\n");
					
					return 1;
				}
			}
		}
	}
	
	return -1;
	
}
