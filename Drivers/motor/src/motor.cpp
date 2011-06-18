#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "roboard.h"
#include "pwm.h"

#include "motor.h"


int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "motor");	//inits the driver

	/* Messages and services */

	ros::NodeHandle motorN;

	/* Publish */

	/* Subscribe */

	ros::Subscriber sub1 = motorN.subscribe("pidRampFront",	100, frontCallback);
	ros::Subscriber sub2 = motorN.subscribe("pidRampLeft",	100, leftCallback);
	ros::Subscriber sub3 = motorN.subscribe("pidRampRight",	100, rightCallback);
	ros::Subscriber sub4 = motorN.subscribe("pidRampBack",	100, backCallback);

	//ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	initMotors();

	while (ros::ok()){
		ros::spin();
	}

	printf("Shutting Down\n");

	return 0;
}

/*************************************************
** Returns the PWM rate for the front motor	**
*************************************************/

void frontCallback(const std_msgs::Float32::ConstPtr& pidRampFront){
	frontPWM = pidRampFront->data;
	frontPWM *= SCALAR;
	frontUPWM = (unsigned int)frontPWM;
	frontUPWM += ZERO_DUTY_CYCLE_US;
	
	updatePWM(FRONT_MOTOR_CHANNEL, frontUPWM);
	return;
}

/*************************************************
** Returns the PWM rate for the left motor	**
*************************************************/

void leftCallback(const std_msgs::Float32::ConstPtr& pidRampLeft){
	leftPWM = pidRampLeft->data;
	leftPWM *= SCALAR;
	leftUPWM = (unsigned int)leftPWM;
	leftUPWM += ZERO_DUTY_CYCLE_US;
	
	updatePWM(LEFT_MOTOR_CHANNEL, leftUPWM);
	return;
}

/*************************************************
** Returns the PWM rate for the right motor	**
*************************************************/

void rightCallback(const std_msgs::Float32::ConstPtr& pidRampRight){
	rightPWM = pidRampRight->data;
	rightPWM *= SCALAR;
	rightUPWM = (unsigned int)rightPWM;
	rightUPWM += ZERO_DUTY_CYCLE_US;
	
	updatePWM(RIGHT_MOTOR_CHANNEL, rightUPWM);
	return;
}

/*************************************************
** Returns the PWM rate for the back motor	**
*************************************************/

void backCallback(const std_msgs::Float32::ConstPtr& pidRampBack){
	backPWM = pidRampBack->data;
	backPWM *= SCALAR;
	backUPWM = (unsigned int)backPWM;
	backUPWM += ZERO_DUTY_CYCLE_US;
	
	updatePWM(BACK_MOTOR_CHANNEL, backUPWM);
	return;
}

/*************************************************
** Initialises the motors			**
*************************************************/

void initMotors(void){
        // Set the channels to produce a zero velocity PWM
        pwm_SetPulse( LEFT_MOTOR_CHANNEL, PWM_FREQUENCY_US, (ZERO_DUTY_CYCLE_US + LEFT_PWM_OFFSET));
        pwm_SetPulse( RIGHT_MOTOR_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + RIGHT_PWM_OFFSET );
        pwm_SetPulse( FRONT_MOTOR_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + FRONT_PWM_OFFSET );
        pwm_SetPulse( BACK_MOTOR_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + BACK_PWM_OFFSET );

        pwm_SetCountingMode( LEFT_MOTOR_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( RIGHT_MOTOR_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( FRONT_MOTOR_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( BACK_MOTOR_CHANNEL, PWM_CONTINUE_MODE );
        
        // Enable the pins
        pwm_EnablePin( LEFT_MOTOR_CHANNEL );
        pwm_EnablePin( FRONT_MOTOR_CHANNEL );
        pwm_EnablePin( RIGHT_MOTOR_CHANNEL );
        pwm_EnablePin( BACK_MOTOR_CHANNEL );

	ROS_INFO("Motors Initialised");

        pwm_EnableMultiPWM(0xffffffffL);

	ROS_INFO("Motors Online");

	return;
}

/*************************************************
** updates the PWM rate				**
*************************************************/

void updatePWM(unsigned int channel, unsigned int rate){
	unsigned int tmpOffset;

	switch(channel){
		case	FRONT_MOTOR_CHANNEL:	tmpOffset = FRONT_PWM_OFFSET;	break;
		case	LEFT_MOTOR_CHANNEL:	tmpOffset = LEFT_PWM_OFFSET;	break;
		case	RIGHT_MOTOR_CHANNEL:	tmpOffset = RIGHT_PWM_OFFSET;	break;
		case	BACK_MOTOR_CHANNEL:	tmpOffset = BACK_PWM_OFFSET;	break;
		default: ROS_ERROR("Dude this is not a valid PWM channel");	break;
	}

	if(!pwm_SetPulse(channel, PWM_FREQUENCY_US, (rate + tmpOffset))){
		ROS_ERROR("Failed to update PWM");
	}

	return;
}
