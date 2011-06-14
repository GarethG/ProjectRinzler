#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h> 
#include <fcntl.h> 

#include "ros/ros.h"
#include "sonarDriver.h"

int fd; 							/* File descriptor for the port */
unsigned char returnBuffer[10000]; 	/*Buffer which stores read data*/
unsigned char *rBptr;				/*Ptr*/
char sendBuffer[13];


int buffLen, 	//length of the recieved buffer, output from read_port()
	i,			//counter
	temp[263],
	length,
	msgLen;
	
unsigned int header,	//Message Header. 
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

	open_port();
	config_port();
	
	while(cmd != 1)
	{
		if(sortPacket() == 1)
		{
			cmd = returnMsg();
			printf("%d\n", cmd);
		
			if(cmd == 4)
			{
				makePacket(1);
				write_port();
			}
		}
		
	}	
	close(fd);
	return 0;
}

/*********************************
** Opens serial port S0		**
*********************************/

int open_port(void){	

	fd = open("/dev/ttyUSB2", O_RDWR | O_NDELAY | O_NOCTTY);
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

	cfsetispeed(&options, BAUDRATE);
	cfsetospeed(&options, BAUDRATE);

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
	//char buffer[]={0x00,0x05,0x04,0xbf,0x71};
	//char buffer[]={0x00,0x05,0x01,0xef,0xd4};

	n = write(fd,sendBuffer,sizeof(sendBuffer));

	if (n < 0){
		ROS_ERROR("Failed to write to port");
		return 0;
	}
	else{
		printf("We Transmitted %d\n",n);
	}
	return (n);
}

/*************************************************
** Reads data off of the serial port and will	**
** then return the data into returnBuffer	**
*************************************************/

int read_port(void){	

	int n;

	n = read(fd, returnBuffer,sizeof(returnBuffer));

	//printf("We read %d bytes\n",n);

	//printf("\n\n");

	rBptr = &returnBuffer[0];

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
'@'		:	Message Header.
HHHH	:	Hex Length of whole binary packet (excluding LF Terminator).
BB		:	Binary Word of above Hex Length.
SID		:	Packet Source Identification (Tx Node number 0 - 255).
DID		:	Packet Destination Identification (Rx Node number 0 -255).
COUNT	:	Byte Count of attached message that follows this byte.
MSG		:	Command / Reply Message (i.e. 'Alive' command, 'Data Reply' message).
TERM	:	Message Terminator = Line Feed (0Ah).
* 
* 
* This needs something to take care of varying length msgs then should 
* be working at least with the alive command, then see at how to make
* it deal with all others.
* 
* 
* 	Returns -1 if failed, 1 if correct first time, 2 if it had to 
* 	stich packets
* 
****************************************************************/
int sortPacket(void)
{

	int packetFlag = 0;
	
	//How long was the msg, according to read() ?
	buffLen = read_port();
		
	rBptr = &returnBuffer[0];	//set pointer for recieved data
			
	//Store all packets into temp[]	
	
	for( i = 0; i < buffLen; i++ )
	{
		temp[i] = getU8();
	}

	//Store temp stuff into right place
	
	header = temp[0];
	hLength = getU32(temp[1], temp[2], temp[3], temp[4]);
	bLength = getU16(temp[6], temp[5]);
	sID = temp[7];
	dID = temp[8];
	byteCount = temp[9];
	term = temp[buffLen-1];
	
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
	
	if(header == '@' && term == 10)
	{

		//prinfPacket();
		
		packetFlag = 1;
		
		//printf("%d, %d\n", byteCount, msgLen+1);
	}
	else
	{
		//printf("Bad Packet, I'll try to recover it >.<\n");
	
	/*****************
	 * Should recover when split into two packets, not any more though
	 * ****************/
		
		//should be recoverable as long as we have header and bp1_buffLen is 0
		if( temp[0] == '@' )//&& bp1_buffLen == NULL)
		{
			//printf("01\n");
			
			if(bLength != 0)
			{
				//printf("01.5 %d\n", buffLen);
				for( i = 0; i < buffLen; i++ )
					bp1_temp[i] = temp[i];
				
				bp1_buffLen = buffLen;
				bp1_bLength = bLength;
			}

		}
		//if bp1_buffLen isn't null we are already trying to recover
		else if( temp[buffLen-1] == 10)
		{
			//printf("02 %d = %d\n",buffLen + bp1_buffLen, bp1_bLength + 6);
			
			for( i = bp1_buffLen; i < (bp1_buffLen + buffLen); i++ )
				bp1_temp[i] = temp[i-bp1_buffLen];
			
			if(buffLen + bp1_buffLen == bp1_bLength + 6)
				//printf("Recovered the packet :\n");
				
				
				buffLen = buffLen + bp1_buffLen;
				
				//Store temp stuff into right place
	
				header = bp1_temp[0];
				hLength = getU32(bp1_temp[1], bp1_temp[2], bp1_temp[3], bp1_temp[4]);
				bLength = getU16(bp1_temp[6], bp1_temp[5]);
				sID = bp1_temp[7];
				dID = bp1_temp[8];
				byteCount = bp1_temp[9];
				term = bp1_temp[buffLen-1];
	
				for(i = 0; i < 263; i++)
					msg[i] = NULL;
					
				msgLen = 0;
				//Store the msg, works for varying lengths
				for( i = 10; i < (buffLen-2); i ++)
				{
					
					msg[(i-10)] = bp1_temp[i];
					msgLen ++;
					
				}
				
				//What is prinf?
				
				if(header == '@' && term == 10)
				{
					
					//prinfPacket();
					
					packetFlag = 2;
					
					//printf("%d, %d\n", byteCount, msgLen+1);
				}	
				
			
		}
		else
		{
			//printf("Sorry, my bad. I really dropped the ball on this one (the ball is my packet)\n");
			packetFlag = -1;
		}
		
		
	}
	
	//Flush out temp
	
	for( i = 0; i < buffLen; i++ )
	{
		temp[i] = 0;
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
 * 
 * Sorry this might be discusting ;/
 * *********************************************/
int makePacket(int command)
{
	
	//{0x40, 0x30, 0x30, 0x30, 0x38, 0x08, 0x00, 0xFF, 0x02, 0x03, 0x17, 0x80, 0x02, 0x0A }
	//Header
	sendBuffer[0] = 0x40;			//Static
	//hex length
	sendBuffer[1] = 0x30;
	sendBuffer[2] = 0x30;
	sendBuffer[3] = 0x30;
	sendBuffer[4] = 0x38;
	//binary length
	sendBuffer[5] = 0x08;
	sendBuffer[6] = 0x00;
	//Source/Dest ID
	sendBuffer[7] = 0xFF;			//Static ??
	sendBuffer[8] = 0x02;			//Static ??
	//ByteCount
	sendBuffer[9] = 0x03;
	//MSG
	
	temp[0] = 0x17;
	temp[1] = 0x80;
	temp[2] = 0x02;
	
	for( i = 0; i < sendBuffer[9]; i ++ )
		sendBuffer[i+10] = temp[i];
	
	//Terminator
	sendBuffer[sendBuffer[9]+10] = 0x0A;			//Static
	
	
}

/************************************************
 * 
 *  Clear packet, clears the sendBuffer and 
 * returns a 1 to show it's done. 
 * 
 * *********************************************/
int clearPacket(void)
{
	
	for( i = 0; i < sizeof(sendBuffer); i ++)
		sendBuffer[i] = 0;
		
	return 1;
}

/**********************************************
 * Prinf the packet
 * *******************************************/
void prinfPacket(void)
{
	
	printf("%d : ", buffLen);
	printf("%d ", header);
	printf("| %d - %d |", hLength, bLength);
	printf(" %d |", byteCount);
	printf(" %d |\n\n", term);
					
}
/**********************************************
 * Find the length of the packet
 * Flag = 0 - sendBuffer
 * Flag = 1 - recvBuffer
 * Returns packet length, -1 if error.
 * *******************************************/
int packetLength(int flag)
{
	
	int len, temp;
	//sendBuffer
	if(flag == 0)
	{
	
		//Check for a 0 and i the value before is 0x10 it's the end of the packet
		while(1)
		{
			while( sendBuffer[len] != 0 )
			{
				len++;
				
				if( sendBuffer[len] == 0 && temp == 0x10 )
					return len;
				
				temp = sendBuffer[len];
			}
		}

	}
	//rcvBuffer
	else if(flag == 1)
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