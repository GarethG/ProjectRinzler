/* Defines */

#define HLSHIGH	80
#define	HLSLOW	60
#define GREY	10

/* Globals */

/* Image Names */
IplImage *cv_input_;
IplImage*	camStream = NULL;
IplImage*	hlsStream = NULL;
IplImage*	hlsStream2 = NULL;
IplImage*	greyStream = NULL;

cv::Mat img_in_;
cv::Mat img_out_;

/* Storage Areas */
CvMemStorage*	hlsStorage = NULL;
CvMemStorage*	greyStorage = NULL;

/* Camera handle */ 
CvCapture 	*capture = 0;

/* Function Declerations */

void obtainOrange(void);
void findCentre(void);


