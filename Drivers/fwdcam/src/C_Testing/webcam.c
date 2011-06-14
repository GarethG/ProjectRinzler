#include "cv.h"
#include "highgui.h"
#include <stdlib.h>
#include <stdio.h>

#define DELAY 2

int main( int argc, char** argv )
{
  // Set up variables
  char window_name[] = "Webcam Test";
  CvCapture *capture = 0;
  IplImage *frame = 0;
  int quit = 0;

  //set the camera capture
  capture = cvCaptureFromCAM(0);

  // create a new window with auto sizing
  cvNamedWindow(window_name, 1);

while(quit != 'q')
{

	//grab frame from webcam
	frame = cvQueryFrame(capture);


    	// display frame
    	cvShowImage(window_name,frame);

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
