#include <stdio.h>   
#include <unistd.h>  
#include <fcntl.h>   
#include <errno.h>   
//#include <termios.h> 
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"


using namespace std;

	ros::init(argc, argv, "SonarLoc");	//inits the driver

	/* Messages and services */

	ros::NodeHandle sonarLocN;

	/* Publish */

	ros::Publisher sonarLocHeadingMsg = sonarLocN.advertise<std_msgs::Int16>("sonarLocCMD", 100);//Publish a generic command to the sonar

	/*Sets up the message structures*/

	std_msgs::int16_t sonarLocCMD;


	ros::Subscriber sub1 = pilotN.subscribe("compassHeading", 100, headingCallback);
//	ros::Subscriber sub2 = pilotN.subscribe("SonarArray", 100, SonarCallback);
//	ros::Subscriber sub3 = pilotN.subscribe("svpDepth", 100, depthCallback); // if out of water stop? if near surface might be getting bad readings - should change threshold?
//	ros::Subscriber sub4 = pilotN.subscribe("compassPitch", 100, pitchCallback); compensate trigonometry? 
	
	ROS_INFO("Sonar Loc Is Online");

//////////////////////////////////////////////////////////////////////////////////////////////////
//Subcribe -SVP: SVP // calculate distance 

//Subcribe -Sonar_Driver: gradients, binIndex // also later scale in meters and number of samples
//Publish -Sonar_Driver: X(leftlim+ engage stare),Y(leftlim+ engage stare), 360 scan when holding pos? >> gmapper?

//Subcribe - Waypoint_manager: current waypoint // does not exist - make simulated input
//Publish - Waypoint_manager: X, Y

//Subcribe - Compass: Current Heading

//Publish - Control: output theta & thruster step, required full stop and end theta
//////////////////////////////////////////////////////////////////////////////////////////////////
//current position >> difference >> Target waypoint >> size of/ number of actual thruster 'steps'

//250 BINS
//BINS 0 to 255
//Bearing 0 to 5760 * (1/16) , is 5760 max?

// scale * binIndex / totalSamples
// 20Meters * bin55 / 200 samples

//clockwise, zero degrees behind



int main( int argc, char **argv )
{
char wait; 
float grad = 3000;
float bearing = 0;
int scale = 20;
int samples = 200;
int binIndex = 55;
int range = 0;

ros::Rate loop_rate(1); // what speed is this?

	while(ros::ok()){

	ros::spinOnce();
	
	range = ((scale * binIndex) / samples);
	bearing = ( grad * 0.0625);

	//cout << "Range: " << range << " Meters" << endl;
	//cout << "Bearing: " << bearing << " Degrees" << endl;
	//	cout << "press any character and enter to continue" << endl;
	//	cin >> wait;

	}// end ROS while
	return 0; 
}// end main










