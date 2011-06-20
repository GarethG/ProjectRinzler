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

	/*Sets up the message structures*/

	std_msgs::Float32 pilotHeading;
	std_msgs::Float32 pilotDepth;
	std_msgs::Float32 pilotPitch;
	std_msgs::Float32 pilotGo;


	/* Subscribe */

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
			pilotHeading.data = 0.0f;
			pilotDepth.data = 0.0f;
			pilotPitch.data = 0.0f;

			ROS_DEBUG("PH: %.3f PD: %.3f PP: %.3f GO: %.1f",pilotHeading.data,pilotDepth.data,pilotPitch.data,pilotGo.data);

			/*Below here we publish our readings*/
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
