/* Defines */

#define	DEPTHP	0.8		//percentage given to depth
#define	PITCHP	1.0 - DEPTHP	//percentage given to pitch

/* Globals */

float backRateDepth, backRatePitch;

unsigned int depthChange;

/* Function Declerations */

void depthCallback(const std_msgs::Float32::ConstPtr& backDRate);
void pitchCallback(const std_msgs::Float32::ConstPtr& backPRate);
