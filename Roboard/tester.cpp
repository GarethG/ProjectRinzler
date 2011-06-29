#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "roboard.h"
#include "pwm.h"

#include "motor.h"

//#define DEBUG
//#define RELEASE

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	char c[100],s[100],e[100];
	unsigned int speed;
	int counter = 100;

	if(!initMotors()){
		printf("Failed to get a root lock\n");
		return 0;
	}
	else{
		printf("Root lock ok\n");
	}

	printf("Enter your motor channel\n\nChoices are: left right fwd back\n\n");
	scanf("%s",&c);
	if(!strcmp(c,"left") || !strcmp(c,"right") || !strcmp(c,"fwd") || !strcmp(c,"back")){
		printf("You chose %s\n",c);
	}
	else{
		printf("Please pick correctly\n");
		return 0;
	}

	printf("Please enter your speed\n\nMax reverse speed is 1000\nMax forward speed is 2000\nZero speed is 1500\n\n");
	scanf("%s",&s);
	speed = atoi(s);
	if(speed < MIN_DUTY_CYCLE_US || speed > MAX_DUTY_CYCLE_US){
		printf("Bad Speed Entered\n");
		return 0;
	}

	printf("Selected Speed: %u\n",speed);

	/*while(strcmp(e,"Y")){
		printf("Would you like to end? Y to end\n");
		scanf("%s",&e);
	}*/
	while(counter){
		printf("%d Loops Remain\n",counter);
		if(!strcmp(c,"left")){
			updatePWM(LEFT_MOTOR_CHANNEL, speed);
		}
		else if(!strcmp(c,"right")){
			updatePWM(RIGHT_MOTOR_CHANNEL, speed);
		}
		else if(!strcmp(c,"fwd")){
			updatePWM(FRONT_MOTOR_CHANNEL, speed);
		}
		else if(!strcmp(c,"back")){
			updatePWM(BACK_MOTOR_CHANNEL, speed);
		}
		counter--;
		usleep(10000);
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
** Initialises the motors			**
*************************************************/

int initMotors(void){

	if(!pwm_Initialize(0xffff, PWMCLOCK_50MHZ, PWMIRQ_DISABLE)){
        	//ROS_ERROR("Unable to initialise PWM library - %s", roboio_GetErrMsg());
		return 0;
	}
	else{
		//ROS_INFO("Successfully logged in as root, pwm activated");
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

	//ROS_INFO("Motors Initialised");

        pwm_EnableMultiPWM(0xffffffffL);

	//ROS_INFO("Motors Online");

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
		default: break;//ROS_ERROR("Dude this is not a valid PWM channel");	break;
	}

	if(rate > MAX_DUTY_CYCLE_US){
		rate = MAX_DUTY_CYCLE_US;
		//ROS_WARN("Attempting to over speed motor %u",channel);
	}
	if(rate < MIN_DUTY_CYCLE_US){
		rate = MIN_DUTY_CYCLE_US;
		//ROS_WARN("Attempting to over reverse motor %u",channel);
	}

	if(!pwm_SetPulse(channel, PWM_FREQUENCY_US, (rate + tmpOffset))){
		//ROS_ERROR("Failed to update PWM");
	}

	return;
}
