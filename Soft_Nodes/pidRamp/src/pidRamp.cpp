#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"

#include "pidRamp.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "pidRamp");

	/* Messages and Services */

	ros::NodeHandle pidRampN;

	/* Publish */

	ros::Publisher frontMsg = pidRampN.advertise<std_msgs::UInt32>("pidRampFront", 100);
	ros::Publisher leftMsg = pidRampN.advertise<std_msgs::UInt32>("pidRampLeft", 100);
	ros::Publisher rightMsg = pidRampN.advertise<std_msgs::UInt32>("pidRampRight", 100);
	ros::Publisher backMsg = pidRampN.advertise<std_msgs::UInt32>("pidRampBack", 100);

	std_msgs::UInt32 pidRampFront;
	std_msgs::UInt32 pidRampLeft;
	std_msgs::UInt32 pidRampRight;
	std_msgs::UInt32 pidRampBack;

	/* Subscribe */

	ros::Subscriber sub1 = pidRampN.subscribe("frontRate", 100, frontRateCallback);
	ros::Subscriber sub2 = pidRampN.subscribe("leftRate", 100, leftRateCallback);
	ros::Subscriber sub3 = pidRampN.subscribe("rightRate", 100, rightRateCallback);
	ros::Subscriber sub4 = pidRampN.subscribe("backRate", 100, backRateCallback);
	ros::Subscriber sub5 = pidRampN.subscribe("pilotSpeed", 100, speedCallback);

	ros::Rate loop_rate(25);

	ROS_INFO("PID Rate Online");

	while(ros::ok()){

		ros::spinOnce();

		pidRampFront.data = slewer(FRONT);
		pidRampLeft.data = slewer(LEFT);
		pidRampRight.data = slewer(RIGHT);
		pidRampBack.data = slewer(BACK);

		//ROS_DEBUG("F: %u L: %u R: %u B: %u",pidRampFront.data,pidRampLeft.data,pidRampRight.data,pidRampBack.data);

		frontMsg.publish(pidRampFront);
		leftMsg.publish(pidRampLeft);
		rightMsg.publish(pidRampRight);
		backMsg.publish(pidRampBack);

		loop_rate.sleep();

	}

	printf("Shutting Down\n");

	return 0;
}


/*************************************************
** Get front pwm target value			**
*************************************************/

void frontRateCallback(const std_msgs::Float32::ConstPtr& frontRate){
	targetRate[FRONT] = (int)frontRate->data;
	return;
}

/*************************************************
** Get left pwm target value			**
*************************************************/

void leftRateCallback(const std_msgs::Float32::ConstPtr& leftRate){
	targetRate[LEFT] = (int)leftRate->data;
	return;
}

/*************************************************
** Get right pwm target value			**
*************************************************/

void rightRateCallback(const std_msgs::Float32::ConstPtr& rightRate){
	targetRate[RIGHT] = (int)rightRate->data;
	return;
}

/*************************************************
** Get back pwm target value			**
*************************************************/

void backRateCallback(const std_msgs::Float32::ConstPtr& backRate){
	targetRate[BACK] = (int)backRate->data;
	return;
}

/*************************************************
** Returns the pilots speed			**
*************************************************/

void speedCallback(const std_msgs::Float32::ConstPtr& pilotSpeed){
	speed = pilotSpeed->data;
	return;
}

/*************************************************
** Slides the current PWM value until it is	**
** matching the target PWM value. This should	**
** prevent the snapping motion seen before 	**
** which damaged one of the motor pins		**
*************************************************/

unsigned int slewer(unsigned int pos){
	
	if(first){
		currentRate[pos] = 0;//targetRate[pos]; //could be why the pwm wasn't starting (i.e. ramp too fast)
		first = 0;
	}
	if(targetRate[pos] == 0){
		currentRate[pos] = 0;
	}
	else{
		if(targetRate[pos] > currentRate[pos]){
			if(pos == FRONT){
				if((currentRate[pos] + 10) > targetRate[pos]){
					currentRate[pos] += 1;
				}
				else{
					currentRate[pos] += CATCHRATEF;
				}	
			}
			else{
				if((currentRate[pos] + 10) > targetRate[pos]){
					currentRate[pos] += 1;
				}
				else{
					currentRate[pos] += CATCHRATE;
				}
			}
		}
		else if(targetRate[pos] < currentRate[pos]){
			if(pos == FRONT){
				/*if((currentRate[pos] - 10) < targetRate[pos]){
					currentRate[pos] -= 1;
				}
				else{
					currentRate[pos] -= CATCHRATEF;
				}*/
				currentRate[pos] = targetRate[pos];
			}
			else{
				if((currentRate[pos] - 10) < targetRate[pos]){
					currentRate[pos] -= 1;
				}
				else{
					currentRate[pos] -= CATCHRATE;
				}
			}
		}
	}

	if((pos == LEFT) || (pos == RIGHT)){

		if(speed == 0.0){
			if(currentRate[pos] > MAXSPEED){
				currentRate[pos] = MAXSPEED;
			}
	
			if(currentRate[pos] < MINSPEED){
				currentRate[pos] = MINSPEED;
			}
		}
		else{
			if(currentRate[pos] > (MAXSPEED + speed)){
				currentRate[pos] = (MAXSPEED + speed);
			}
	
			if(currentRate[pos] < MINSPEED){
				currentRate[pos] = MINSPEED;
			}

			if(currentRate[pos] > 100.0){
				currentRate[pos] = 100.0;
			}
		}
	}
	else{
		if(currentRate[pos] > MAXSPEEDD){
			currentRate[pos] = MAXSPEEDD;
		}

		if(currentRate[pos] < MINSPEEDD){
			currentRate[pos] = MINSPEEDD;
		}
	}		

	outRate[pos] = currentRate[pos];
	outRate[pos] *= SCALAR;
	outRate[pos] = outRate[pos] + ZERO_DUTY_CYCLE_US;

	ROS_DEBUG("Chan %u Speed %d Target %d Out %d",pos,currentRate[pos],targetRate[pos],outRate[pos]);

	return (unsigned int)outRate[pos];
}
