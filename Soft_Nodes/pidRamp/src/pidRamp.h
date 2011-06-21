/* Defines */

#define CATCHRATE	0.1
#define MAXSPEED	30.0
#define	MINSPEED	-30.0

#define	FRONT		0
#define	LEFT		1
#define RIGHT		2
#define BACK		3

/* Globals */

float targetRate[4];
float currentRate[4];
unsigned int first = 1;

/* Function Declerations */

void frontRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
void leftRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
void rightRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
void backRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
float slewer(unsigned int pos);
