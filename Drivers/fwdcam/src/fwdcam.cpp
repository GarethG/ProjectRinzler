/*
 *  OpenCV Demo for ROS
 *  Copyright (C) 2010, I Heart Robotics
 *  I Heart Robotics <iheartrobotics@gmail.com>
 *  http://www.iheartrobotics.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include "fwdcam.h"

// ROS/OpenCV HSV Demo
// Based on http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

class Demo{

	protected:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		sensor_msgs::CvBridge bridge_;

		cv::Mat img_hsv_;
		cv::Mat img_hue_;
		cv::Mat img_sat_;
		cv::Mat img_bin_;



	public:

		Demo (ros::NodeHandle & nh):nh_ (nh), it_ (nh_){
			// Listen for image messages on a topic and setup callback
			image_sub_ = it_.subscribe ("/gscam/image_raw", 1, &Demo::imageCallback, this);
			// Open HighGUI Window
			cv::namedWindow ("input", 1);
			cv::namedWindow ("hsl", 1);
			cv::namedWindow ("output", 1);
	}

	void imageCallback (const sensor_msgs::ImageConstPtr & msg_ptr){
		// Convert ROS Imput Image Message to IplImage
		try{
			cv_input_ = bridge_.imgMsgToCv (msg_ptr, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException error){
			ROS_ERROR ("CvBridge Input Error");
		}

		// Convert IplImage to cv::Mat
		img_in_ = cv::Mat (cv_input_).clone ();
		// output = input
		img_out_ = img_in_.clone ();
		
		obtainOrange();

		// Display Input image
		cv::imshow ("input", cv_input_);
		// Display Binary Image
		cv::imshow ("hsl", hlsStream);
		// Display segmented image
		cv::imshow ("output", greyStream);

		// Needed to  keep the HighGUI window open
		cv::waitKey (3);
	}

};

void obtainOrange(void){
	ROS_INFO("Orange1");
	if(hlsStream == NULL){
		hlsStream = cvCreateImage(cvGetSize(cv_input_), 8, 3);		//create the HLS channel
		hlsStorage = cvCreateMemStorage(0);				//make a space
	}
	else{	
		cvClearMemStorage(hlsStorage);					//clear previous frame
	}
	ROS_INFO("Orange2");
	cvCvtColor(cv_input_, hlsStream, CV_BGR2HLS);				//creates HLS colour
	ROS_INFO("Orange3");
	//cvThreshold(hlsStream, hlsStream, HLSHIGH, 255, CV_THRESH_BINARY);	//threshold up
	ROS_INFO("Orange4");
	//cvThreshold(hlsStream, hlsStream, HLSLOW, 255, CV_THRESH_BINARY);	//threshold down
	ROS_INFO("Orange5");
	if(greyStream == NULL ){						//create bw channel
		greyStream = cvCreateImage(cvGetSize(hlsStream), 8, 1);
		greyStorage = cvCreateMemStorage(0);
	}
	else{
		cvClearMemStorage(greyStorage);
	}
	/*cvCvtColor(cv_input_, greyStream, CV_RGB2GRAY);			//convert colour to bw
	//cvThreshold(greyStream, greyStream, GREY, 255, CV_THRESH_BINARY);	//threshold*/
	img_in_ = cv::Mat (hlsStream).clone ();
	cv::threshold(img_in_, img_in_, HLSHIGH, 255, cv::THRESH_BINARY);	//threshold up
	ROS_INFO("Orange6");
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


int main(int argc, char **argv){
	// Initialize ROS Node
	ros::init (argc, argv, "ihr_demo1");
	// Start node and create a Node Handle
	ros::NodeHandle nh;
	// Instaniate Demo Object
	ROS_INFO("Online");
	Demo d (nh);
	// Spin ...
	ros::spin ();
	// ... until done
	return 0;
}
