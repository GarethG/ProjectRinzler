#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "pilot.h"

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	int counter;

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

	ros::Subscriber sub1 = pilotN.subscribe("compassHeading", 100, headingCallback);
	ros::Subscriber sub2 = pilotN.subscribe("svpDepth", 100, depthCallback);
	ros::Subscriber sub3 = pilotN.subscribe("compassPitch", 100, pitchCallback);

	//ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	ROS_INFO("Pilot Is Online");


	ros::Rate loop_rate(1);

	counter = 10;

	while(ros::ok()){

		ros::spinOnce();

		if(counter > 0){

			ROS_INFO("We go in %d seconds",counter);

			counter--;

			if(counter == 0){
				pilotGo.data = 1.0;
				pilotGoMsg.publish(pilotGo);
			}
			else{
				pilotGo.data = 0.0;
				pilotGoMsg.publish(pilotGo);
			}
		}
		else{
			pilotHeading.data = 90.0f;
			pilotDepth.data = 0.0f;
			pilotPitch.data = 0.0f;
			pilotSpeed.data = 10.0f;

			ROS_DEBUG("PH: %.3f PD: %.3f PP: %.3f GO: %.1f",pilotHeading.data,pilotDepth.data,pilotPitch.data,pilotGo.data);

			/*Below here we publish our readings*/
			pilotSpeedMsg.publish(pilotSpeed);
			pilotGoMsg.publish(pilotGo);
			pilotHeadingMsg.publish(pilotHeading);		
			pilotDepthMsg.publish(pilotDepth);		
			pilotPitchMsg.publish(pilotPitch);
		}
		loop_rate.sleep();
	}

	printf("Shutting Down\n");

	return 0;
}

/*************************************************
** Returns the compass heading			**
*************************************************/

void headingCallback(const std_msgs::Float32::ConstPtr& compassHeading){
	heading = compassHeading->data;
	return;
}


/*************************************************
** Returns the compass pitch			**
*************************************************/

void pitchCallback(const std_msgs::Float32::ConstPtr& compassPitch){
	pitch = compassPitch->data;
	return;
}


/*************************************************
** Returns the svp depth			**
*************************************************/

void depthCallback(const std_msgs::Float32::ConstPtr& svpDepth){
	depth = svpDepth->data;
	return;
}
