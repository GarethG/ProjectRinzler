/* Defines */

/* Heading */
#define KPH 2.0
#define KDH 1.0
#define KIH 0.1
/* Depth */
#define KPD 300.0
#define KDD 0.1
#define KID 0.1
/* Pitch */
#define KPP 10.0
#define KDP 1.5
#define KIP 0.1
/* Others */
#define MAXSPEEDH	50.0
#define	MINSPEEDH	-50.0
#define MAXVAL		50.0
#define MINVAL		-50.0
#define FRONTTHRESH	0.8

#define PLUSBUFFH	5.0
#define	MINUSBUFFH	-5.0

#define BUFFTHRESH	10

#define PLUSBUFFD	5.0
#define	MINUSBUFFD	-5.0

/* Globals*/

float heading=0.0;
float targetHeading=0.0;
float pitch=0.0;
float targetPitch=0.0;
float depth=0.0;
float targetDepth=0.0;
float speed=0.0;
unsigned int go = 0;

float left;
float right;

float KP,KD,KI;

int PLUSBUFF, MINUSBUFF;
int deadZ =0;
int buffCheck = 0;

/* Function Declerations */

float correctError(float error);
void goCallback(const std_msgs::UInt32::ConstPtr& pilotGo);
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
