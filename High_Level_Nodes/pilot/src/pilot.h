/* Defines */
//#define POOL
#define RUN

#ifdef RUN

#define RUNDEPTH 	11.3
#define RUNSPEED	80.0
#define HACC		7.0
#define HCOUNT		10

/* state 0*/
#define FIRSTHEADING	190.0

/* state 1 */
#define OUTTIME		90

/* state 2 */
#define SECONDHEADING	10.0

/* state 3 */
#define FTHRESH		4

/* state 4 */
#define THIRDHEADING	280.0

/* state 5 */
#define WALLRANGE	4
#define WALLACC		0.5

/* state 6 */
#define FOURTHHEADING	190.0

#endif

#ifdef POOL

#define RUNDEPTH 	10.7
#define RUNSPEED	30.0
#define HACC		7.0
#define HCOUNT		10

/* state 0*/
#define FIRSTHEADING	190.0

/* state 1 */
#define OUTTIME		20

/* state 2 */
#define SECONDHEADING	10.0

/* state 3 */
#define FTHRESH		1

/* state 4 */
#define THIRDHEADING	280.0

/* state 5 */
#define WALLRANGE	1
#define WALLACC		0.5

/* state 6 */
#define FOURTHHEADING	190.0

#endif

/* default */

#define	STOPSPEED	-10.0
#define	STOPHEADING	290.0
#define	STOPDEPTH	10.2

/* Globals*/

float theading=0.0;
float heading=0.0;
float pitch=0.0;
float depth=0.0;
float fRange = 0.0;
float rRange = 0.0;

unsigned int switcher=0;
unsigned int tcounter=0;
unsigned int hcounter=0;

unsigned int go = 0;
unsigned int latch = 0;
unsigned int done = 0;

/* Function Declerations */

void headingCallback(const std_msgs::Float32::ConstPtr& compassHeading);
void pitchCallback(const std_msgs::Float32::ConstPtr& compassPitch);
void depthCallback(const std_msgs::Float32::ConstPtr& svpDepth);
void adcGoCallback(const std_msgs::UInt32::ConstPtr& adcGo);
void sonarFCallback(const std_msgs::Float32::ConstPtr& sonarFront);
void sonarDCallback(const std_msgs::UInt32::ConstPtr& sonarDone);
void sonarRCallback(const std_msgs::Float32::ConstPtr& sonarRight);
