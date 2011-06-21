/* Defines */



/* Globals*/

float go=0.0;
float heading=0.0;
float pitch=0.0;
float depth=0.0;
float speed=0.0;

/* Function Declerations */

void goCallback(const std_msgs::Float32::ConstPtr& keyGo);
void headingCallback(const std_msgs::Float32::ConstPtr& keyHeading);
void pitchCallback(const std_msgs::Float32::ConstPtr& keyPitch);
void depthCallback(const std_msgs::Float32::ConstPtr& keyDepth);
void speedCallback(const std_msgs::Float32::ConstPtr& keySpeed);
