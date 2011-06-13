/* Defines */

#define	PWM_FREQUENCY_US	20000
#define	MIN_DUTY_CYCLE_US	1000
#define	MAX_DUTY_CYCLE_US	2000
#define	ZERO_DUTY_CYCLE_US	(MIN_DUTY_CYCLE_US + MAX_DUTY_CYCLE_US)/2

#define	RIGHT_MOTOR_CHANNEL	3
#define	LEFT_MOTOR_CHANNEL	5
#define	FRONT_MOTOR_CHANNEL	4
#define	BACK_MOTOR_CHANNEL	2

#define	LEFT_PWM_OFFSET		0
#define	RIGHT_PWM_OFFSET	-20
#define	FRONT_PWM_OFFSET	0
#define	BACK_PWM_OFFSET		-20

/* Globals */

float 	frontPWM,
	leftPWM,
	rightPWM,
	backPWM;

/* Function Declarations */

void updatePWM(unsigned int channel, unsigned int rate);
void frontCallback(const std_msgs::Float32::ConstPtr& pidRampFront);
void leftCallback(const std_msgs::Float32::ConstPtr& pidRampLeft);
void rightCallback(const std_msgs::Float32::ConstPtr& pidRampRight);
void backCallback(const std_msgs::Float32::ConstPtr& pidRampBack);
void initMotors(void);
