/* Defines */

#define CATCHRATE	1
#define MAXSPEED	50
#define	MINSPEED	-50

#define	FRONT		0
#define	LEFT		1
#define RIGHT		2
#define BACK		3

/* Globals */

unsigned int targetRate[4];
unsigned int currentRate[4];
unsigned int first = 1;

/* Function Declerations */

void frontRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
void leftRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
void rightRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
void backRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
unsigned int slewer(unsigned int pos);

