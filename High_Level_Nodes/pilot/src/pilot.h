/* Defines */



/* Globals*/

float heading=0.0;
float pitch=0.0;
float depth=0.0;

/* Function Declerations */

void headingCallback(const std_msgs::Float32::ConstPtr& compassHeading);
void pitchCallback(const std_msgs::Float32::ConstPtr& compassPitch);
void depthCallback(const std_msgs::Float32::ConstPtr& svpDepth);
