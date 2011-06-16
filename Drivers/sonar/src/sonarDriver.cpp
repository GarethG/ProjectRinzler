#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h> 
#include <fcntl.h> 
#include <sys/time.h>

#include "ros/ros.h"
#include "sonarDriver.h"

int fd; 							/* File descriptor for the port */
unsigned char returnBuffer[10000]; 	/*Buffer which stores read data*/
unsigned char *rBptr;				/*Ptr*/
char sendBuffer[18];


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
		
		//makeSendData();

		if(initSonar() == 1)
		{

			//Make and send headcommand
			makeHeadPacket();
			write_port();
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
					
					//Send mtSendData command
					//log mtHeadData into a file

						//makeSendData();
						while(1)
						{
							makePacket(mtSendData);
							write_port();
							makePacket(mtSendData);
							write_port();
							printf(">> mtSendData\n");
							
							read_port();
							
							printf("\n-- %d --\n", returnBuffer[10]);
							
							if(sortPacket() == 1)
							{
								
								cmd = returnMsg();
								printf("-- %d --\n", cmd);
								if(cmd == mtHeadData)
								{
									printf("\t<< mtHeadData!!\n");
									while(1)
									{
										
									}
								}
								
							}
						}
					
				}							
				

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

	fd = open("/dev/ttyUSB0", O_RDWR | O_NDELAY | O_NOCTTY);
	printf("/dev/ttyUSB0\n");
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
		//printf("We Transmitted %d\n",n);
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
	//printf("buffLen = %d\n", buffLen);
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
	
	if(header == '@' && term == 10)
	{

		//prinfPacket();
		
		packetFlag = 1;
		
		
		if(msg[0] == mtHeadData || msg[0] == mtAlive)
		{
			
			for(i = 0; i < buffLen; i++ )
				printf("%x : ", temp[i]);
			printf("\n"
			);
			
		}
		
		//printf("%d, %d\n", byteCount, msgLen+1);
	}
	else
	{
		packetFlag = -1;
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
void makePacket(int command)
{
	
	clearPacket();
	
	if(command == 16 || command == 23 || command == 24)
	{
		sendBuffer = {0x40, 0x30, 0x30, 0x30, 0x38, 0x08, 0x00, 0xFF, 0x02, 0x03, command, 0x80, 0x02, 0x0A };
	}
	else if(command == 25)
	{
		
		int time0, time1, time2, time3;
		
		struct timeval tv;

		gettimeofday(&tv, NULL); 
		
		int seconds = tv.tv_sec % 60;
		int minutes = tv.tv_sec % 3600;
		int hours = tv.tv_sec % 86400;
		
		//printf("%d:%d:%d.%d -- ", hours / 3600 , minutes / 60 , seconds, tv.tv_usec / 10000);
		
		int ms = ( (hours + minutes + seconds) * 1000 ) + (tv.tv_usec / 10000);
		
		char buff[30];
		
		//printf("%x -- ", ms);
		sprintf(buff, "%x", ms);
		
		to_hex(buff, 0);

		for (i = 1; buff[i] != '\0'; i += 2)
			//printf("%#x ", (unsigned char)buff[i]);
			
		//printf("\n");
		
		time0 = buff[0];
		time1 = buff[1];
		time2 = buff[2];
		time3 = buff[3];
		
		sendBuffer = {0x40, 0x30, 0x30, 0x30, 0x43, 0x0C, 0x00, 0xFF, 0x02, 0x07, command, 0x80, 0x02, time0, time1, time2, time3, 0x0A };
		
	}
	else
	{
		printf("Parp.\n");
	}
/*		//Header
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
		
		temp[0] = command;
		temp[1] = 0x80;
		temp[2] = 0x02;
		
		for( i = 0; i < sendBuffer[9]; i ++ )
		{
			sendBuffer[i+10] = temp[i];
		}
		
		//Terminator
		sendBuffer[sendBuffer[9]+10] = 0x0A;			//Static
		
*/
	
}

/************************************************
 * 
 *  make a packet for sending mtHeadCommand
 * *********************************************/
void makeHeadPacket(void)
{
	
	//{0x40, 0x30, 0x30, 0x30, 0x38, 0x08, 0x00, 0xFF, 0x02, 0x03, 0x17, 0x80, 0x02, 0x0A }
	//Header
	sendBuffer[0] = 0x40;			//Static
	//hex length
	sendBuffer[1] = 0x30;
	sendBuffer[2] = 0x30;
	sendBuffer[3] = 0x34;
	sendBuffer[4] = 0x43;
	//binary length
	sendBuffer[5] = 0x4C;
	sendBuffer[6] = 0x00;
	//Source/Dest ID
	sendBuffer[7] = 0xFF;			//Static ??
	sendBuffer[8] = 0x02;			//Static ??
	//ByteCount
	sendBuffer[9] = 0x47;
	//MSG
		//Command
		sendBuffer[10] = 0x13;
		//
		sendBuffer[11] = 0x80;
		sendBuffer[12] = 0x02;
		//mtHeadCommand Type; 1 = Normal, 29 = with appended V3B Gain Params for Dual Channel
		sendBuffer[13] = 0x01;
	//Head Parameter Info.
			//HdCtrl bytes
			sendBuffer[14] = 0x83;
			sendBuffer[15] = 0x23;
	//HdType
			sendBuffer[16] = 0x02;
	//TxN/RxN Transmitter Constants
			//TxN Channel 1
			sendBuffer[17] = 0x99;
			sendBuffer[18] = 0x99;		
			sendBuffer[19] = 0x99;
			sendBuffer[20] = 0x02;	
			//TxN Channel 2
			sendBuffer[21] = 0x66;
			sendBuffer[22] = 0x66;		
			sendBuffer[23] = 0x66;
			sendBuffer[24] = 0x05;	
			//RxN Channel 1
			sendBuffer[25] = 0xA3;
			sendBuffer[26] = 0x70;		
			sendBuffer[27] = 0x3D;
			sendBuffer[28] = 0x06;	
			//RxN Channel 2
			sendBuffer[29] = 0x70;
			sendBuffer[30] = 0x3D;		
			sendBuffer[31] = 0x0A;
			sendBuffer[32] = 0x09;
	//TxPulseLen
			sendBuffer[33] = 0x28;
			sendBuffer[34] = 0x00;
	//RangeScale
			sendBuffer[35] = 0x3C;
			sendBuffer[36] = 0x00;
	//LeftAngleLimit
			sendBuffer[37] = 0x01;
			sendBuffer[38] = 0x00;
	//RightAngleLiimit
			sendBuffer[39] = 0xFF;
			sendBuffer[40] = 0x18; 
	//ADSpan
			sendBuffer[41] = 0x51; 
	//ADLow
			sendBuffer[42] = 0x08;
	//IGain Setting
			sendBuffer[43] = 0x54;
			sendBuffer[44] = 0x54; 
	//SlopeSetting
			sendBuffer[45] = 0x5A;
			sendBuffer[46] = 0x00;
			sendBuffer[47] = 0x7D;
			sendBuffer[48] = 0x00;
	//MoTime
			sendBuffer[49] = 0x19;
	//StepAngleSize
			sendBuffer[50] = 0x10;
	//ADInterval
			sendBuffer[51] = 0x8D;
			sendBuffer[52] = 0x00;
	//Description of NBins
			sendBuffer[53] = 0x5A;
			sendBuffer[54] = 0x00;
	//MaxADbuf
			sendBuffer[55] = 0xE8;
			sendBuffer[56] = 0x03;
	//Lockout
			sendBuffer[57] = 0x97;
			sendBuffer[58] = 0x03;
	//MinorAxisDirection
			sendBuffer[59] = 0x40;
			sendBuffer[60] = 0x06;
	//MajorAxisDirection
			sendBuffer[61] = 0x01;
	//Ctl2
			sendBuffer[62] = 0x00;
	//ScanZ
			sendBuffer[63] = 0x00;
			sendBuffer[64] = 0x00;	
			
	//for( i = 65; i < 82; i++ )
	//	sendBuffer[i] = 0x00;
			
	//Terminator
	sendBuffer[65] = 0x0A;			//Static

	
}

int makeSendData()
{
	
	struct timeval tv;

	gettimeofday(&tv, NULL); 
	
	int seconds = tv.tv_sec % 60;
	int minutes = tv.tv_sec % 3600;
	int hours = tv.tv_sec % 86400;
	
	printf("%d:%d:%d.%d -- ", hours / 3600 , minutes / 60 , seconds, tv.tv_usec / 10000);
	
	int ms = ( (hours + minutes + seconds) * 1000 ) + (tv.tv_usec / 10000);
	
	char buff[30];
	
	printf("%x -- ", ms);
	sprintf(buff, "%x", ms);
	
	to_hex(buff, 0);

	for (i = 1; buff[i] != '\0'; i += 2)
		printf("%#x ", (unsigned char)buff[i]);
		
	printf("\n");

	
	
	
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

	int i;
	
	printf("%d : ", buffLen);
	printf("%d ", header);
	printf("| %d - %d |", hLength, bLength);
	printf(" %d |", byteCount);
	
	while(msg[i])
	{
		printf("%x : ", msg[i]);
		i++;
	}
	
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
			write_port();
			printf(">> mtSendVersion\n");
			
			//Read Port
			if(sortPacket() == 1)
			{
				
				cmd = returnMsg();
				//printf("%d : ", cmd);
				
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

void to_hex(char buf[], int i)
{
  if (*buf == '\0')
    return;

  to_hex(buf + 2, i + 1);
  buf[1] = strtol(buf, NULL, 16);
  *buf = '\0';
}
