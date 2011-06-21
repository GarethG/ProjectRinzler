#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "pilotKey.h"

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	int flip = 0;

	ros::init(argc, argv, "pilot");	//inits the driver

	/* Messages and services */

	ros::NodeHandle pilotN;

	/* Publish */

	ros::Publisher pilotHeadingMsg = pilotN.advertise<std_msgs::Float32>("pilotHeading", 100);
	ros::Publisher pilotDepthMsg = pilotN.advertise<std_msgs::Float32>("pilotDepth", 100);
	ros::Publisher pilotPitchMsg = pilotN.advertise<std_msgs::Float32>("pilotPitch", 100);
	ros::Publisher pilotGoMsg = pilotN.advertise<std_msgs::Float32>("pilotGo", 100);
	ros::Publisher pilotSpeedMsg = pilotN.advertise<std_msgs::Float32>("pilotSpeed", 100);

	/*Sets up the message structures*/

	std_msgs::Float32 pilotHeading;
	std_msgs::Float32 pilotDepth;
	std_msgs::Float32 pilotPitch;
	std_msgs::Float32 pilotGo;
	std_msgs::Float32 pilotSpeed;


	/* Subscribe */

	ros::Subscriber sub1 = pilotN.subscribe("keyGo", 	100, goCallback);
	ros::Subscriber sub2 = pilotN.subscribe("keyHeading", 	100, headingCallback);
	ros::Subscriber sub3 = pilotN.subscribe("keyDepth", 	100, depthCallback);
	ros::Subscriber sub4 = pilotN.subscribe("keyPitch", 	100, pitchCallback);
	ros::Subscriber sub5 = pilotN.subscribe("keySpeed", 	100, speedCallback);

	//ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	ROS_INFO("Keyboard Pilot Is Online");

	ros::Rate loop_rate(1);

	while(ros::ok()){

		ros::spinOnce();

		if(flip == 0){
			while(go!=1.0){
				ros::spinOnce();
				loop_rate.sleep();
			}
			flip = 1;
		}

		pilotSpeed.data = speed;
		pilotGo.data = go;
		pilotHeading.data = heading;
		pilotPitch.data = pitch;
		pilotDepth.data = depth;

		/*Below here we publish our readings*/
		pilotSpeedMsg.publish(pilotSpeed);
		pilotGoMsg.publish(pilotGo);
		pilotHeadingMsg.publish(pilotHeading);		
		pilotDepthMsg.publish(pilotDepth);		
		pilotPitchMsg.publish(pilotPitch);

		loop_rate.sleep();
	}

	printf("Shutting Down\n");

	return 0;
}

/*************************************************
** Returns the go				**
*************************************************/

void goCallback(const std_msgs::Float32::ConstPtr& keyGo){
	go = keyGo->data;
	return;
}

/*************************************************
** Returns the heading				**
*************************************************/

void headingCallback(const std_msgs::Float32::ConstPtr& keyHeading){
	heading = keyHeading->data;
	return;
}

/*************************************************
** Returns the pitch				**
*************************************************/

void pitchCallback(const std_msgs::Float32::ConstPtr& keyPitch){
	pitch = keyPitch->data;
	return;
}

/*************************************************
** Returns the depth				**
*************************************************/

void depthCallback(const std_msgs::Float32::ConstPtr& keyDepth){
	depth = keyDepth->data;
	return;
}

/*************************************************
** Returns the speed				**
*************************************************/

void speedCallback(const std_msgs::Float32::ConstPtr& keySpeed){
	speed = keySpeed->data;
	return;
}
