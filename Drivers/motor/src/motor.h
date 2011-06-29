/* Defines */

#define	PWM_FREQUENCY_US	20000
#define	MIN_DUTY_CYCLE_US	1000
#define	MAX_DUTY_CYCLE_US	2000
#define	ZERO_DUTY_CYCLE_US	(MIN_DUTY_CYCLE_US + MAX_DUTY_CYCLE_US)/2

#define	RIGHT_MOTOR_CHANNEL	3
#define	LEFT_MOTOR_CHANNEL	5
#define	FRONT_MOTOR_CHANNEL	4
#define	BACK_MOTOR_CHANNEL	6
#define TEST_CHANNEL		1

#define	LEFT_PWM_OFFSET		0
#define	RIGHT_PWM_OFFSET	0
#define	FRONT_PWM_OFFSET	0
#define	BACK_PWM_OFFSET		0

#define	DEBUGSPEED		1800

/* Globals */

unsigned int 	frontPWM,
		leftPWM,
		rightPWM,
		backPWM;

unsigned int go = 0, go2 = 0, go3 = 0, go4 = 0;

/* Function Declarations */

void goCallback(const std_msgs::UInt32::ConstPtr& pilotGo);
void updatePWM(unsigned int channel, unsigned int rate);
void frontCallback(const std_msgs::UInt32::ConstPtr& pidRampFront);
void leftCallback(const std_msgs::UInt32::ConstPtr& pidRampLeft);
void rightCallback(const std_msgs::UInt32::ConstPtr& pidRampRight);
void backCallback(const std_msgs::UInt32::ConstPtr& pidRampBack);
int initMotors(void);
