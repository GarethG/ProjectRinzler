#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "fwdcam.h"



int main(int argc, char **argv){ 			//we need argc and argv for the rosInit function

	ros::init(argc, argv, "camera");		//inits the driver
	ros::NodeHandle n;				//this is what ROS uses to connect to a node

	ros::Rate loop_rate(2); 			//how many times a second (i.e. Hz) the code should run

	ROS_INFO("Camera Online");			//solve the crimes with the cameras

	capture = cvCaptureFromCAM(0);			//open camera

	while (ros::ok()){	

		camStream = cvQueryFrame(capture);	//obtain webcam image

		obtainOrange();				//find orange

		findCentre();				//find centre

		/*Have a snooze*/

		loop_rate.sleep();

	}

	cvReleaseCapture(&capture);

	printf("Shutting Down\n");

	return 0;
}

void obtainOrange(void){

	printf("Here\n");

	cvGetSize(camStream);

	printf("Here now\n");

	if(hlsStream == NULL){
		hlsStream = cvCreateImage(cvGetSize(camStream), 8, 3);		//create the HLS channel
		hlsStorage = cvCreateMemStorage(0);				//make a space
	}
	else{	
		cvClearMemStorage(hlsStorage);					//clear previous frame
	}

	printf("Here2\n");

	cvCvtColor(camStream, hlsStream, CV_RGB2HLS);				//creates HLS colour
	cvThreshold(hlsStream, hlsStream, HLSHIGH, 255, CV_THRESH_BINARY_INV);	//threshold up
	cvThreshold(hlsStream, hlsStream, HLSLOW, 255, CV_THRESH_BINARY);	//threshold down
	
	if(greyStream == NULL ){						//create bw channel
		greyStream = cvCreateImage(cvGetSize(hlsStream), 8, 1);
		greyStorage = cvCreateMemStorage(0);
	}
	else{
		cvClearMemStorage(greyStorage);
	}
	cvCvtColor(greyStream, greyStream, CV_RGB2GRAY);			//convert colour to bw
	cvThreshold(greyStream, greyStream, GREY, 255, CV_THRESH_BINARY);	//threshold

	return;
}

void findCentre(void){
	int x, y, count, x_estimate, y_estimate, x_centre, y_centre;
	CvScalar s;

	x_estimate = y_estimate = count = 0;
	
	for(x=0;x<greyStream->height;x++){
		for(y=0;y<greyStream->width;y++){
			s = cvGet2D(greyStream,x,y);
			if(s.val[0] == 0){		//if we have found a black pixel
				count++;		//increase counters
				x_estimate += x;
				y_estimate += y;
			}
		}
	}

	if(count < 10000){				//arbritrary size threshold
		printf("No Target in sight saw only %d\n",count);
	}
	else{
		x_centre = y_estimate / count;		//estimate center of x
		y_centre = x_estimate / count;		//and y

		printf("X: %d Y: %d Count: %d Yeahhhhhhhhhhhhhhhhhhhhhh buoy!\n",x_centre,y_centre,count);
	}

	return;
}	
