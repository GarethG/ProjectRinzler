/* Defines */

#define KP 0.01
#define KD 0.1
#define KI 0.1

#define MAXVAL	20.0
#define MINVAL	-20.0


/* Globals*/

float heading;
float targetHeading;

/* Function Declerations */

void headingCallback(const std_msgs::Float32::ConstPtr& compassHeading);
void targetCallback(const std_msgs::Float32::ConstPtr& pilotHeading);
float p(void);
float pd(void);
float pi(void);
float pid(void);
