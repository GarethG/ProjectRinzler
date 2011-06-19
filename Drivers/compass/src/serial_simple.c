#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <math.h>
#include <string.h>

/*
* 'open_port()' - Open serial port 1.
*
* Returns the file descriptor on success or -1 on error.
*/

int fd; /* File descriptor for the port */
unsigned char returnBuffer[100];
unsigned char *rBptr;

int open_port(void){	

	fd = open("/dev/ttyS0", O_RDWR | O_NDELAY | O_NOCTTY);
	if (fd == -1){
		perror("open_port: Unable to open /dev/ttyS0 - ");
	}
	else{
		//fcntl(fd, F_SETFL, 0);
		fcntl(fd, F_SETLK, 0);
		printf("Port opened %d\n",fd);
	}
	return (fd);
}

void config_port(void){
	struct termios options;

	tcgetattr(fd, &options);

	cfsetispeed(&options, B19200);
	cfsetospeed(&options, B19200);

	options.c_cflag |= (CLOCAL | CREAD | CS8);

	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME] = 10;
	options.c_cc[VMIN] = 0;
	tcflush(fd, TCIFLUSH);

	tcsetattr(fd, TCSANOW, &options);

	printf("configured\n");

	return;
}


int write_port(void){
	int n;
	unsigned char buffer[]={0x00,0x05,0x04,0xbf,0x71};
	//unsigned char buffer[]={0x00,0x05,0x01,0xEF,0xD4};

	//char buffer[] = "Trololololololololololololol";

/*	printf("We are about to transmit the string %s\n\n",buffer);

	printf("We are about to transmit the buffer: ");

	for(n=0;n<sizeof(buffer);n++){
		printf("%x",buffer[n]);
		if(n!=(sizeof(buffer)-1)){
			printf(":");
		}
	}

	printf("\n\n");*/

	n = write(fd,buffer,sizeof(buffer));

	//n = write(fd,buff,sizeof(buff));
	if (n < 0){
		fputs("write() of 4 bytes failed!\n", stderr);
	}

	printf("written %d bytes\n",n);

	return (n);
}

int read_port(void){	
	int n,i;

	printf("read attempt\n");

	for(i=0;i<sizeof(returnBuffer);i++){
		returnBuffer[i] = 'X';
	}

	n = read(fd,returnBuffer,sizeof(returnBuffer));

	//printf("As a string: %s\n\nAs a returnBuffer: ",returnBuffer);

	for(i=0;i<n;i++){
		//printf("I read %c at %d\n",returnBuffer[i],i);
		if(i!=0){
			printf(":");
		}
		printf("%x",returnBuffer[i]);
		
	}

	printf("\n");

	printf("read %d bytes\n",n);

	return n;
}

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

unsigned int getU16(void){

	unsigned int tmp1, tmp2;

	tmp1 = *rBptr++;	//get the first item
	tmp2 = *rBptr++;	//get the second item

	//printf("tmp1: %u tmp2: %u ",tmp1,tmp2);

	tmp1 <<= 8;		//shift 8 bits left

	tmp1 |= tmp2;		//or to combine data

	//printf("result: %u\n",tmp1);

	return tmp1;		//return the U16
}

unsigned int getU8(void){
	return *rBptr++;	//returns the current point on the array and shifts along (a U8)
}

float getF32(void){
	unsigned int tmp1;

	tmp1 = getU32();
	
	return *((float*)&tmp1);
}
	

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
	

int main(void){
	int size;

	fd = open_port();

	config_port();

	while(1){
	
		size = write_port();
	
		//printf("We transmitted %d bytes\n\n",size);

		//sleep(1);

		size = read_port();
	
		//printf("We read %d bytes\n\n",size);

		if(size > 5){
			parseBuffer();
		}
		sleep(1);
	}

	close(fd);
	printf("Port Closed\n\n");

	

	return 0;
}
