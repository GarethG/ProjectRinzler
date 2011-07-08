#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"

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
	ros::Publisher pilotOkGoMsg = pilotN.advertise<std_msgs::UInt32>("pilotOkGo", 100);
	ros::Publisher pilotSpeedMsg = pilotN.advertise<std_msgs::Float32>("pilotSpeed", 100);
	ros::Publisher alertFrontMsg = pilotN.advertise<std_msgs::UInt32>("alertFront", 100);

	/*Sets up the message structures*/

	std_msgs::Float32 pilotHeading;
	std_msgs::Float32 pilotDepth;
	std_msgs::Float32 pilotPitch;
	std_msgs::UInt32 pilotOkGo;
	std_msgs::Float32 pilotSpeed;
	std_msgs::UInt32 alertFront;


	/* Subscribe */

	ros::Subscriber sub1 = pilotN.subscribe("compassHeading", 100, headingCallback);
	ros::Subscriber sub2 = pilotN.subscribe("svpDepth", 100, depthCallback);
	ros::Subscriber sub3 = pilotN.subscribe("compassPitch", 100, pitchCallback);
	ros::Subscriber sub4 = pilotN.subscribe("adcGo", 	100, adcGoCallback);

	//ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	ROS_INFO("Pilot Is Online");


	ros::Rate loop_rate(1);

	counter = 10;

	while(ros::ok()){

		ros::spinOnce();

		if(latch){
			go = 1;
		}

		if((counter > 0) && go){

			ROS_INFO("We go in %d seconds",counter);

			counter--;

			if(counter == 0){
				pilotOkGo.data = 1;
				pilotOkGoMsg.publish(pilotOkGo);
			}
			else{
				if(counter > 7){
					alertFront.data = 1750;
					alertFrontMsg.publish(alertFront);
				}
				else{
					alertFront.data = 1500;
					alertFrontMsg.publish(alertFront);
				}

				pilotOkGo.data = 0;
				pilotOkGoMsg.publish(pilotOkGo);
			}
		}
		else if(go){
			switch(switcher){
				case	0:	pilotHeading.data = FIRSTHEADING;
						pilotDepth.data = RUNDEPTH;
						pilotPitch.data = 0.0f;
						pilotSpeed.data = -10.0f;

						if((heading < (FIRSTHEADING + HACC)) && (heading > (FIRSTHEADING - HACC))){
							if(hcounter > HCOUNT){
								switcher++;
								tcounter=0;
								hcounter = 0;
							}
							else{
								hcounter++;
							}
						}
						break;

				case	1:	pilotHeading.data = FIRSTHEADING;
						pilotDepth.data = RUNDEPTH;
						pilotPitch.data = 0.0f;
						pilotSpeed.data = RUNSPEED;

						if(tcounter >= OUTTIME){
							switcher++;
							tcounter=0;
						}
						break;
	
				case	2:	/*pilotHeading.data = SECONDHEADING;
						pilotDepth.data = RUNDEPTH;
						pilotPitch.data = 0.0f;
						pilotSpeed.data = -10.0f;

						if((heading < (SECONDHEADING + HACC)) && (heading > (SECONDHEADING - HACC))){
							if(hcounter > HCOUNT){
								switcher++;
								tcounter=0;
								hcounter = 0;
							}
							else{
								hcounter++;
							}
						}
						break;

				case	3:	pilotHeading.data = SECONDHEADING;
						pilotDepth.data = RUNDEPTH;
						pilotPitch.data = 0.0f;
						pilotSpeed.data = RUNSPEED;

						if(tcounter >= INTIME){
							switcher++;
							tcounter=0;
						}
						break;

				case 	4:	*/pilotHeading.data = STOPHEADING;
						pilotDepth.data = STOPDEPTH;
						pilotPitch.data = 0.0f;
						pilotSpeed.data = STOPSPEED;
						if(tcounter >= 4){
							pilotOkGo.data = 0;
						}
						break;
				default:	ROS_ERROR("WRONG!");	break;
			}
			
			
			tcounter++;
			ROS_DEBUG("PH: %.3f PD: %.3f PP: %.3f GO: %u State: %u Count: %u H %u",pilotHeading.data,pilotDepth.data,pilotPitch.data,pilotOkGo.data, switcher,tcounter,hcounter);

			/*Below here we publish our readings*/
			pilotSpeedMsg.publish(pilotSpeed);
			pilotOkGoMsg.publish(pilotOkGo);
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

/*************************************************
** Returns the ADC				**
*************************************************/

void adcGoCallback(const std_msgs::UInt32::ConstPtr& adcGo){
	go = adcGo->data;
	
	if(go){
		latch = 1;
	}

	return;
}
