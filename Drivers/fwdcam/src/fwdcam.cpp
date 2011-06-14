#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>

/*#include "cv.h"
#include "highgui.h"*/
#include <stdlib.h>
#include <stdio.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DELAY 2

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "camera");	//inits the driver
	ros::NodeHandle n;			//this is what ROS uses to connect to a node

	ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	ROS_INFO("Camera Online");	//solve the crimes with the cameras

	// Set up variables
	char window_name[] = "Webcam Test";
	CvCapture *capture = 0;
	IplImage *frame = 0;
	int quit = 0;

	//set the camera capture
	capture = cvCaptureFromCAM(1);

	printf("ok\n");

	// create a new window with auto sizing
	cvNamedWindow(window_name, 1);
	
	printf("ok\n");

	while (ros::ok()){	

		printf("%d\n",quit++);

		/*Have a snooze*/

		//ros::spinOnce();

		//grab frame from webcam
		frame = cvQueryFrame(capture);


		// display frame
		cvShowImage(window_name,frame);

		loop_rate.sleep();

	}

	// before quitting it releases memory
	cvReleaseCapture(&capture);
	cvDestroyWindow(window_name);

	printf("Shutting Down\n");

	return 0;
}
