#include <sys/types.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

char buf[20];
size_t nbytes;
ssize_t bytes_read;
int fd;

int main(void){


	fd=open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);

	if(fd == -1 ){
		perror("open_port: Unable to open /dev/ttyS0 – ");

	}
	else{
		fcntl(fd, F_SETFL,0);
		printf("Port 1 has been sucessfully opened and %d is the file description\n",fd);
	}

	while(1){
		nbytes = sizeof(buf);
		bytes_read = read(fd, buf, nbytes);

		printf("I read %s\n",buf);

		sleep(1);

	}

	return 0;
}
