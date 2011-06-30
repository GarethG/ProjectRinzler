#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */



#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "compassDriver.h"


//global for accessing the serial port.
int serialPort;

int main(int argc, char **argv){ //we need argc and argv for the rosInit function
	
	char *returnPtr;

	struct termios tio;
        struct termios stdio;
        int tty_fd;
        fd_set rdset;
	unsigned char c='D';

	ros::init(argc, argv, "compass");	//inits the driver
	ros::NodeHandle n;			//this is what ROS uses to connect to a node

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("output", 1000);	/*advertise publishes a message, informs of the type of data
											and what the data is known by (i.e. its name). From here we
											also define the size of the buffer, so if we are producing
											data faster than we can send a large buffer is needed, else
											a smaller buffer should suffice */

	ros::Rate loop_rate(1); //how many times a second (i.e. Hz) the code should run

	stdio.c_iflag=0;
        stdio.c_oflag=0;
        stdio.c_cflag=0;
        stdio.c_lflag=0;
        stdio.c_cc[VMIN]=1;
        stdio.c_cc[VTIME]=0;
        tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
        tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking
 
 
 
 
        memset(&tio,0,sizeof(tio));
        tio.c_iflag=0;
        tio.c_oflag=0;
        tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
        tio.c_lflag=0;
        tio.c_cc[VMIN]=1;
        tio.c_cc[VTIME]=5;
 
        tty_fd=open("/dev/ttyS0", O_RDWR | O_NONBLOCK);      
        cfsetospeed(&tio,B38400);            // 115200 baud
        cfsetispeed(&tio,B38400);            // 115200 baud
 
        tcsetattr(tty_fd,TCSANOW,&tio);

	/*if(!openPort()){
		printf("Port Open Failure\n");
		return 0;
	}
	
	printf("Starting PNI\n");

	if(!startPni()){
		printf("Port Write Failed\n");
		return 0;
	}

	printf("PNI started\n");
*/
	//int count = 0;
	while (ros::ok()){
		/*std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);

		ros::spinOnce();

		
		++count;*/

		/*returnPtr = strtok(readPni(),"&");		//reads serial and cuts off any garbage present
		printf("Read: %s\n",returnPtr);*/
		
		if (read(tty_fd,&c,1)>0)        write(STDOUT_FILENO,&c,1);              // if new data is available on the serial port, print it out
                //if (read(STDIN_FILENO,&c,1)>0)  write(tty_fd,&c,1);                     // if new data is available on the console, send it to the serial port


		loop_rate.sleep();

	}


	return 0;
}

/*****************************************
** Opens the port to allow data to be 	**
** read by the software. Prints to 	**
** standard error in case of failure	**
*****************************************/
int openPort(void){

	serialPort = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (serialPort == -1){
		perror("open_port: Unable to open /dev/ttyS0 - ");
	}

	return (serialPort);
}

/*****************************************
** Sends the go command to the compass	**
*****************************************/

int startPni(void){
	return write(serialPort, "00 05 04 bf 71", 13);
}

/*****************************************
** Reads from the serial port and 	**
** places a & at the end to ensure any	**
** garbage is removed from the string	**
*****************************************/

char *readPni(void){
	
	char returnBuffer[1000];
	int ok;

	ok = read(serialPort,returnBuffer,1000);	//reads data into the buffer stores the number of characters read into ok

	if(!ok){
		return "";
	}
	else{
		strcat(returnBuffer,"&");
		return returnBuffer;
	}
}
