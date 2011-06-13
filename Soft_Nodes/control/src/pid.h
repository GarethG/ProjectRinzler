/* Defines */

#define KP 0.01
#define KD 0.1
#define KI 0.1

#define MAXVAL		30.0
#define MINVAL		00.0
#define FWDHACK		10.0
#define FRONTTHRESH	20.0




/* Globals*/

float heading;
float targetHeading;
float pitch;
float targetPitch;
float depth;
float targetDepth;

/* Function Declerations */

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
