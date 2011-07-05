#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include <string>

void xCallback(const std_msgs::Int32::ConstPtr& logX);
void yCallback(const std_msgs::Int32::ConstPtr& logY);
void zCallback(const std_msgs::Int32::ConstPtr& logZ);
void aCallback(const std_msgs::String::ConstPtr& logAction);

int x, y, z;
char aa[100];

int main(int argc, char **argv)
{
    time_t ltime; /* calendar time */
    ltime=time(NULL); /* get current cal time */
	
	ros::init(argc, argv, "logger");
	
	ros::NodeHandle n;
	
	ros::Subscriber sub1 = n.subscribe("logX", 100, xCallback);
	ros::Subscriber sub2 = n.subscribe("logY", 100, yCallback);
	ros::Subscriber sub3 = n.subscribe("logZ", 100, zCallback);
	ros::Subscriber sub4 = n.subscribe("logAction", 100, aCallback);

	ros::spinOnce();

	while (ros::ok())
	{

		ltime=time(NULL); /* get current cal time */

		printf("%s,%d.x,%d.y,%d.z,%s.aa\n",asctime( localtime(&ltime) ), x, y, z, aa);
		
		ros::spinOnce();
		
		sleep(10);
	}
	
}

/*************************************************
** Returns the sonar bearing **
*************************************************/

void xCallback(const std_msgs::Int32::ConstPtr& logX)
{
	x = logX->data;
	return;
}

/*************************************************
** Returns the sonar bearing **
*************************************************/

void yCallback(const std_msgs::Int32::ConstPtr& logY)
{
	y = logY->data;
	return;
}
/*************************************************
** Returns the sonar bearing **
*************************************************/

void zCallback(const std_msgs::Int32::ConstPtr& logZ)
{
	z = logZ->data;
	return;
}
/*************************************************
** Returns the sonar bearing **
*************************************************/

void aCallback(const std_msgs::String::ConstPtr& logAction)
{
	//aa = logAction->data;

	strcpy(aa, (logAction->data.c_str()));
	
	//strcpy(char * dest, const char * src);
	
	//logAction ->data;
	

	return;
}
