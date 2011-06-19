/* Defines */

#define KP 0.01
#define KD 0.1
#define KI 0.1

#define MAXVAL		30.0
#define MINVAL		00.0
#define FRONTTHRESH	20.0

/* Globals*/

float heading=0.0;
float targetHeading=0.0;
float pitch=0.0;
float targetPitch=0.0;
float depth=0.0;
float targetDepth=0.0;
float speed=0.0;
float go = 0.0;

/* Function Declerations */

float correctError(float error);
void goCallback(const std_msgs::Float32::ConstPtr& goNode);
void speedCallback(const std_msgs::Float32::ConstPtr& pilotSpeed);
void headingCallback(const std_msgs::Float32::ConstPtr& compassHeading);
void targetHeadingCallback(const std_msgs::Float32::ConstPtr& pilotHeading);
void pitchCallback(const std_msgs::Float32::ConstPtr& compassPitch);
void targetPitchCallback(const std_msgs::Float32::ConstPtr& pilotPitch);
void depthCallback(const std_msgs::Float32::ConstPtr& svpDepth);
void targetDepthCallback(const std_msgs::Float32::ConstPtr& pilotDepth);
float p(float value, float targetValue);
float pd(float value, float targetValue);
float pi(float value, float targetValue);
float pid(float value, float targetValue);
