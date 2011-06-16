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

void somethingNew(void){

	int x, y, count, x_estimate, y_estimate, x_centre, y_centre;
	int x_draw1, x_draw2, y_draw1, y_draw2;
	CvScalar s;

	x_estimate = y_estimate = count = 0;
	
	for(x=0;x<g_grey->height;x++){
		for(y=0;y<g_grey->width;y++){
			s = cvGet2D(g_grey,x,y);
			if(s.val[0] == 0){
				count++;
				x_estimate += x;
				y_estimate += y;
			}
		}
	}

	if(count == 0){
		printf("No Object in sight\n");
	}
	else{
		x_centre = y_estimate / count;
		y_centre = x_estimate / count;
	}

	x_draw1 = x_centre + 100;
	x_draw2 = x_centre - 100;
	y_draw1 = y_centre + 100;
	y_draw2 = y_centre - 100;

	s.val[0] = 255;
	

	inline CvPoint pt1 = {x_draw1,y_centre};
	inline CvPoint pt2 = {x_draw2,y_centre};
	inline CvPoint pt3 = {x_centre,y_draw1};
	inline CvPoint pt4 = {x_centre,y_draw2};

	cvLine(g_grey, pt1 , pt2, s, 1, 8,0);
	cvLine(g_grey, pt3 , pt4, s, 1, 8,0);

	s.val[0] = 0;
	s.val[1] = 255;
	s.val[2] = 0;

	cvLine(g_gray, pt1 , pt2, s, 1, 8,0);
	cvLine(g_gray, pt3 , pt4, s, 1, 8,0);

	printf("X: %u Y: %u\n",x_centre,y_centre);

	return;
}

	

int main(int argc, char** argv){
	// Set up variables
	

	//set the camera capture
	capture = cvCaptureFromCAM(0);

	// create a new window with auto sizing
	//cvNamedWindow(window_name, 1);

	// create a window
	//cvNamedWindow("HSV Window", CV_WINDOW_AUTOSIZE); 

	// create a window
	//cvNamedWindow("Grey Window", CV_WINDOW_AUTOSIZE);

	// create a window
	cvNamedWindow("Target", CV_WINDOW_AUTOSIZE);  
	cvNamedWindow("Target2", CV_WINDOW_AUTOSIZE);  

	while(quit != 'q'){

		//grab frame from webcam
		g_image = cvQueryFrame(capture);

		// display frame
		//cvShowImage(window_name,g_image);

		on_trackbar(0);

		//cvShowImage("HSV Window", g_gray );

		//cvShowImage("Grey Window", g_grey );

		somethingNew();

		cvShowImage("Target", g_grey );
		cvShowImage("Target2", g_gray );

		/*something();

		cvShowImage("blobOut", g_grey );*/

		//printf("Thresh is %d\n",g_thresh++);
	
		/*if(g_thresh == 255){
			g_thresh = 0;
		}*/

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




