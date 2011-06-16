/* Defines */

#define HLSHIGH	80
#define	HLSLOW	60
#define GREY	10

/* Globals */

/* Image Names */
IplImage*	camStream = NULL;
IplImage*	hlsStream = NULL;
IplImage*	greyStream = NULL;

/* Storage Areas */
CvMemStorage*	hlsStorage = NULL;
CvMemStorage*	greyStorage = NULL;

/* Camera handle */ 
CvCapture 	*capture = 0;

/* Function Declerations */

void obtainOrange(void);
void findCentre(void);


