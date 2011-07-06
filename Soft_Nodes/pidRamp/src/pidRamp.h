/* Defines */

#define CATCHRATE		20
#define MAXSPEED		50
#define	MINSPEED		-50

#define	FRONT			0
#define	LEFT			1
#define RIGHT			2
#define BACK			3

#define	SCALAR			5
#define	MIN_DUTY_CYCLE_US	1000
#define	MAX_DUTY_CYCLE_US	2000
#define	ZERO_DUTY_CYCLE_US	(MIN_DUTY_CYCLE_US + MAX_DUTY_CYCLE_US)/2

/* Globals */

int targetRate[4];
int currentRate[4];
int outRate[4];
unsigned int first = 1;

/* Function Declerations */

void frontRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
void leftRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
void rightRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
void backRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
unsigned int slewer(unsigned int pos);

