/* Defines */

#define CATCHRATE	0.01
#define MAXSPEED	20.0
#define	MINSPEED	-20.0

/* Globals */

float targetRate;
float currentRate;
unsigned int first = 1;

/* Function Declerations */

void rateCallback(const std_msgs::Float32::ConstPtr& pidRate);
float slewer(void);
