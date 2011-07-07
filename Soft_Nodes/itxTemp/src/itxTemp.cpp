#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/String.h"

int main(int argc, char **argv)
{

	int tempRead = 0;
	FILE *fp;
	//extern FILE *popen();
	char buff[512];
	
	ros::init(argc, argv, "itxTemp");
	
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<std_msgs::String>("temp", 100);
	
	std_msgs::String temp;
	
	ros::Rate loop_rate(1);
	
	while (ros::ok())
	{

		/* popen creates a pipe so we can read the output
		of the program we are invoking */
		if ( ! ( fp = popen( "sensors | grep temp", "r" ) ) ) 
		{
			exit(1);
		}

		/* read the output of netstat, one line at a time */
		while (fgets(buff, sizeof(buff), fp) != NULL ) 
		{
			printf("Output: %s", buff);
		}

		for(int i = 0; i < 1; i++)
		{
			std::stringstream ss;
			ss << buff;
			temp.data = ss.str();
		}
		
		

		pub.publish(temp);
		
		loop_rate.sleep();
		
	}
	
	/* close the pipe */
	pclose(fp);
	
}
