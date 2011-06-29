#include "ros/ros.h"
#include "std_msgs/UInt32.h"

#include "roboard.h"
#include "pwm.h"

#include "motor.h"

//#define DEBUG
#define RELEASE

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	unsigned char once = 1;
	ros::init(argc, argv, "motor");	//inits the driver

	#ifdef DEBUG
	ROS_WARN("Warning, motors are in debug mode");
	#endif

	if(!initMotors()){
		ROS_ERROR("Root lock failure");
		return 0;
	}

	/* Messages and services */

	ros::NodeHandle motorN;

	/* Publish */

	/* Subscribe */

	ros::Subscriber sub1 = motorN.subscribe("pidRampFront",	100, frontCallback);
	ros::Subscriber sub2 = motorN.subscribe("pidRampLeft",	100, leftCallback);
	ros::Subscriber sub3 = motorN.subscribe("pidRampRight",	100, rightCallback);
	ros::Subscriber sub4 = motorN.subscribe("pidRampBack",	100, backCallback);
	ros::Subscriber sub5 = motorN.subscribe("pilotGo",	100, goCallback);

	ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	while (ros::ok()){
		if(once){
			while(go != 1){
				ros::spinOnce();
				ROS_WARN("Motors waiting for go");
				ros::Duration(1.0).sleep();
			}
			ROS_INFO("Motors given the go");
			loop_rate.sleep();
			once = 0;
		}
		ros::spin();
	}

	updatePWM(RIGHT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
	updatePWM(LEFT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
	updatePWM(FRONT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
	updatePWM(BACK_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
	//updatePWM(TEST_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);

	printf("Shutting Down\n");

	return 0;
}

/*************************************************
** Returns the go signal			**
*************************************************/

void goCallback(const std_msgs::UInt32::ConstPtr& pilotGo){
	go4 = go3 = go2 = go = pilotGo->data;
	return;
}

/*************************************************
** Returns the PWM rate for the front motor	**
*************************************************/

void frontCallback(const std_msgs::UInt32::ConstPtr& pidRampFront){
	frontPWM = pidRampFront->data;
	frontPWM *= SCALAR;
	frontPWM += ZERO_DUTY_CYCLE_US;

	ROS_DEBUG("Front: %u",frontPWM);
	if(go == 1){
		#ifdef DEBUG
		frontPWM = DEBUGSPEED;
		#endif
		updatePWM(FRONT_MOTOR_CHANNEL, frontPWM);
		go = 0;
	}
	else{
		updatePWM(FRONT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
	}
	return;
}

/*************************************************
** Returns the PWM rate for the left motor	**
*************************************************/

void leftCallback(const std_msgs::UInt32::ConstPtr& pidRampLeft){
	leftPWM = pidRampLeft->data;
	leftPWM *= SCALAR;
	leftPWM += ZERO_DUTY_CYCLE_US;

	ROS_DEBUG("Left: %u",leftPWM);
	
	if(go2 == 1){
		#ifdef DEBUG
		leftPWM = DEBUGSPEED;
		#endif
		updatePWM(LEFT_MOTOR_CHANNEL, leftPWM);
		go2 = 0;
	}
	else{
		updatePWM(LEFT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
	}
	return;
}

/*************************************************
** Returns the PWM rate for the right motor	**
*************************************************/

void rightCallback(const std_msgs::UInt32::ConstPtr& pidRampRight){
	rightPWM = pidRampRight->data;
	rightPWM *= SCALAR;
	rightPWM += ZERO_DUTY_CYCLE_US;

	ROS_DEBUG("Right: %u",rightPWM);
	
	if(go3 == 1){
		#ifdef DEBUG
		rightPWM = DEBUGSPEED;
		#endif
		updatePWM(RIGHT_MOTOR_CHANNEL, rightPWM);
		go3 = 0;
	}
	else{
		updatePWM(RIGHT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
	}
	return;
}

/*************************************************
** Returns the PWM rate for the back motor	**
*************************************************/

void backCallback(const std_msgs::UInt32::ConstPtr& pidRampBack){
	backPWM = pidRampBack->data;
	backPWM *= SCALAR;
	backPWM += ZERO_DUTY_CYCLE_US;

	ROS_DEBUG("Back: %u",backPWM);
	
	if(go4 == 1){
		#ifdef DEBUG
		backPWM = DEBUGSPEED;
		#endif
		updatePWM(BACK_MOTOR_CHANNEL, backPWM);
		go4 = 0;
	}
	else{
		updatePWM(BACK_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
	}
	return;
}

/*************************************************
** Initialises the motors			**
*************************************************/

int initMotors(void){

	if(!pwm_Initialize(0xffff, PWMCLOCK_50MHZ, PWMIRQ_DISABLE)){
        	ROS_ERROR("Unable to initialise PWM library - %s", roboio_GetErrMsg());
		return 0;
	}
	else{
		ROS_INFO("Successfully logged in as root, pwm activated");
	}

        // Set the channels to produce a zero velocity PWM
        pwm_SetPulse( LEFT_MOTOR_CHANNEL, PWM_FREQUENCY_US, (ZERO_DUTY_CYCLE_US + LEFT_PWM_OFFSET));
        pwm_SetPulse( RIGHT_MOTOR_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + RIGHT_PWM_OFFSET );
        pwm_SetPulse( FRONT_MOTOR_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + FRONT_PWM_OFFSET );
        pwm_SetPulse( BACK_MOTOR_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + BACK_PWM_OFFSET );
        //pwm_SetPulse( TEST_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + BACK_PWM_OFFSET );

        pwm_SetCountingMode( LEFT_MOTOR_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( RIGHT_MOTOR_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( FRONT_MOTOR_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( BACK_MOTOR_CHANNEL, PWM_CONTINUE_MODE );
        //pwm_SetCountingMode( TEST_CHANNEL, PWM_CONTINUE_MODE );
        
        // Enable the pins
        pwm_EnablePin( LEFT_MOTOR_CHANNEL );
        pwm_EnablePin( FRONT_MOTOR_CHANNEL );
        pwm_EnablePin( RIGHT_MOTOR_CHANNEL );
        pwm_EnablePin( BACK_MOTOR_CHANNEL );
        pwm_EnablePin( TEST_CHANNEL );

	ROS_INFO("Motors Initialised");

        pwm_EnableMultiPWM(0xffffffffL);

	ROS_INFO("Motors Online");

	return 1;
}

/*************************************************
** updates the PWM rate				**
*************************************************/

void updatePWM(unsigned int channel, unsigned int rate){
	unsigned int tmpOffset=0;

	switch(channel){
		case	FRONT_MOTOR_CHANNEL:	tmpOffset = FRONT_PWM_OFFSET;	break;
		case	LEFT_MOTOR_CHANNEL:	tmpOffset = LEFT_PWM_OFFSET;	break;
		case	RIGHT_MOTOR_CHANNEL:	tmpOffset = RIGHT_PWM_OFFSET;	break;
		case	BACK_MOTOR_CHANNEL:	tmpOffset = BACK_PWM_OFFSET;	break;
		//case	TEST_CHANNEL:		tmpOffset = 0;			break;
		default: ROS_ERROR("Dude this is not a valid PWM channel");	break;
	}

	if(rate > MAX_DUTY_CYCLE_US){
		rate = MAX_DUTY_CYCLE_US;
		ROS_WARN("Attempting to over speed motor %u",channel);
	}
	if(rate < MIN_DUTY_CYCLE_US){
		rate = MIN_DUTY_CYCLE_US;
		ROS_WARN("Attempting to over reverse motor %u",channel);
	}

	if(!pwm_SetPulse(channel, PWM_FREQUENCY_US, (rate + tmpOffset))){
		ROS_ERROR("Failed to update PWM");
	}

	return;
}
