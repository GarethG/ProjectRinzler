#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "pidRamp.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "pidRamp");

	/* Messages and Services */

	ros::NodeHandle pidRampN;

	/* Publish */

	ros::Publisher frontMsg = pidRampN.advertise<std_msgs::Float32>("pidRampFront", 100);
	ros::Publisher leftMsg = pidRampN.advertise<std_msgs::Float32>("pidRampLeft", 100);
	ros::Publisher rightMsg = pidRampN.advertise<std_msgs::Float32>("pidRampRight", 100);
	ros::Publisher backMsg = pidRampN.advertise<std_msgs::Float32>("pidRampBack", 100);

	std_msgs::Float32 pidRampFront;
	std_msgs::Float32 pidRampLeft;
	std_msgs::Float32 pidRampRight;
	std_msgs::Float32 pidRampBack;

	/* Subscribe */

	ros::Subscriber sub1 = pidRampN.subscribe("frontRate", 100, frontRateCallback);
	ros::Subscriber sub2 = pidRampN.subscribe("leftRate", 100, leftRateCallback);
	ros::Subscriber sub3 = pidRampN.subscribe("rightRate", 100, rightRateCallback);
	ros::Subscriber sub4 = pidRampN.subscribe("backRate", 100, backRateCallback);


	ros::Rate loop_rate(50);

	ROS_INFO("PID Rate Online");

	while(ros::ok()){

		ros::spinOnce();

		pidRampFront.data = slewer(FRONT);
		pidRampLeft.data = slewer(LEFT);
		pidRampRight.data = slewer(RIGHT);
		pidRampBack.data = slewer(BACK);

		frontMsg.publish(pidRampFront);
		leftMsg.publish(pidRampLeft);
		rightMsg.publish(pidRampRight);
		backMsg.publish(pidRampBack);

		ROS_DEBUG("F: %.3f L: %.3f R: %.3f B: %.3f",pidRampFront.data,pidRampLeft.data,pidRampRight.data,pidRampBack.data);

		loop_rate.sleep();

	}

	printf("Shutting Down\n");

	return 0;
}


/*************************************************
** Get front pwm target value			**
*************************************************/

void frontRateCallback(const std_msgs::Float32::ConstPtr& frontRate){
	targetRate[FRONT] = frontRate->data;
	return;
}

/*************************************************
** Get left pwm target value			**
*************************************************/

void leftRateCallback(const std_msgs::Float32::ConstPtr& leftRate){
	targetRate[LEFT] = leftRate->data;
	return;
}

/*************************************************
** Get right pwm target value			**
*************************************************/

void rightRateCallback(const std_msgs::Float32::ConstPtr& rightRate){
	targetRate[RIGHT] = rightRate->data;
	return;
}

/*************************************************
** Get back pwm target value			**
*************************************************/

void backRateCallback(const std_msgs::Float32::ConstPtr& backRate){
	targetRate[BACK] = backRate->data;
	return;
}

/*************************************************
** Slides the current PWM value until it is	**
** matching the target PWM value. This should	**
** prevent the snapping motion seen before 	**
** which damaged one of the motor pins		**
*************************************************/

float slewer(unsigned int pos){
	
	if(first){
		currentRate[pos] = targetRate[pos];
		first = 0;
	}

	if(targetRate[pos] > currentRate[pos]){
		currentRate[pos] += CATCHRATE;
	}
	else if(targetRate[pos] < currentRate[pos]){
		currentRate[pos] -= CATCHRATE;
	}

	if(currentRate[pos] > MAXSPEED){
		currentRate[pos] = MAXSPEED;
	}

	if(currentRate[pos] < MINSPEED){
		currentRate[pos] = MINSPEED;
	}

	return currentRate[pos];
}
