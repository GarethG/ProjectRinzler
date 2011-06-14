#include "cv.h"
#include "highgui.h"
#include <stdlib.h>
#include <stdio.h>

#define DELAY 2

char window_name[] = "Webcam Test";
CvCapture *capture = 0;

IplImage*	g_image = NULL;
IplImage*	g_gray = NULL;
IplImage*	g_grey = NULL;
int		g_thresh = 80;
int		g_thresh_l = 70;
CvMemStorage* 	g_storage = NULL;
CvMemStorage* 	g_storage2 = NULL;

int quit = 0;

int height,width,step,channels;
uchar *data;
int i,j,k;

void on_trackbar(int a){
	if( g_storage == NULL ){
		g_gray = cvCreateImage( cvGetSize( g_image ), 8, 3 );
		g_storage = cvCreateMemStorage(0);
	} else {
		cvClearMemStorage( g_storage );
	}

	CvSeq* contours = 0;
	cvCvtColor( g_image, g_gray, CV_RGB2HLS);
	cvThreshold( g_gray, g_gray, g_thresh, 255, CV_THRESH_BINARY_INV);
	cvThreshold( g_gray, g_gray, g_thresh_l, 255, CV_THRESH_BINARY);

	if( g_storage2 == NULL ){
		g_grey = cvCreateImage( cvGetSize( g_gray ), 8, 1 );
		g_storage2 = cvCreateMemStorage(0);
	} else {
		cvClearMemStorage( g_storage2 );
	}
	cvCvtColor( g_gray, g_grey, CV_RGB2GRAY);
	//cvThreshold( g_gray, g_gray, g_thresh, 255, CV_THRESH_BINARY_INV);

	return;
}

void something(void){

	CvScalar s;

	height    = g_grey->height;
	width     = g_grey->width;
	step      = g_grey->widthStep;
	channels  = g_grey->nChannels;
	data      = (uchar *)g_grey->imageData;
	//printf("Processing a %dx%d image with %d channels\n",height,width,channels); 

	for(i=0;i<height;i++){
		for(j=0;j<width;j++){

			s=cvGet2D(g_grey,i,j); // get the (i,j) pixel value
	
			if(s.val[0] != 0){
				s.val[0] = 255;
			}

			cvSet2D(g_grey,i,j,s); // set the (i,j) pixel value
		}
	}


	return;
}

int main(int argc, char** argv){
	// Set up variables
	

	//set the camera capture
	capture = cvCaptureFromCAM(0);

	// create a new window with auto sizing
	cvNamedWindow(window_name, 1);

	// create a window
	cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE); 

	// create a window
	cvNamedWindow("blobWin", CV_WINDOW_AUTOSIZE);

	// create a window
	cvNamedWindow("blobOut", CV_WINDOW_AUTOSIZE);  

	while(quit != 'q'){

		//grab frame from webcam
		g_image = cvQueryFrame(capture);

		// display frame
		cvShowImage(window_name,g_image);

		on_trackbar(0);

		cvShowImage("mainWin", g_gray );

		cvShowImage("blobWin", g_grey );

		something();

		cvShowImage("blobOut", g_grey );

		//printf("Thresh is %d\n",g_thresh++);
	
		if(g_thresh == 255){
			g_thresh = 0;
		}

		//check for q to quit
		quit = cvWaitKey(1);

		// delay for a moment, delay is under 2 it doesn't seem to work
		cvWaitKey(DELAY);

	}

	// before quitting it releases memory
	cvReleaseCapture(&capture);
	cvDestroyWindow(window_name);

	return 0;
}




