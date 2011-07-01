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
#include "std_msgs/UInt32.h"

// ROS/OpenCV HSV Demo
// Based on http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

int x_centre, y_centre;

class Demo{

	protected:
		ros::NodeHandle nh_;
		
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		sensor_msgs::CvBridge bridge_;
		cv::Mat img_in_;
		cv::Mat img_hls_;
		cv::Mat img_hsv_;
		cv::Mat img_thresh_hls_;
		cv::Mat img_thresh_hls2_;
		cv::Mat img_grey_;
		cv::Mat img_out_;
		IplImage *cv_input_;

	public:

	Demo (ros::NodeHandle & nh):nh_ (nh), it_ (nh_){
		// Listen for image messages on a topic and setup callback
		image_sub_ = it_.subscribe ("/gscam/image_raw", 1, &Demo::imageCallback, this);
		// Open HighGUI Window
		/*cv::namedWindow ("input", 1);
		cv::namedWindow ("hls", 1);
		cv::namedWindow ("thresh hls up", 1);
		cv::namedWindow ("thresh hls down", 1);
		cv::namedWindow ("grey", 1);
		cv::namedWindow ("out", 1);*/
		//cv::moveWindow("input",200 ,200);
		//cv::namedWindow ("thresh hsv", 1);
	}

	void findCentre(void){
		int x, y, count, x_estimate, y_estimate;
		int x_draw1, x_draw2, y_draw1, y_draw2;
		CvScalar s;

		x_estimate = y_estimate = count = 0;
	
		IplImage ipl_img = img_out_;
		IplImage ipl_hls = img_thresh_hls2_;

		for(x=0;x<ipl_img.height;x++){
			for(y=0;y<ipl_img.width;y++){
				s = cvGet2D(&ipl_img,x,y);
				if(s.val[0] == 0){		//if we have found a black pixel
					count++;		//increase counters
					x_estimate += x;
					y_estimate += y;
				}
			}
		}

		if(count < 3000){				//arbritrary size threshold
			//printf("No Target in sight saw only %d\n",count);
			ROS_WARN("No Target %d",count);
		}
		else{
			x_centre = y_estimate / count;		//estimate center of x
			y_centre = x_estimate / count;		//and y


			x_draw1 = x_centre + 100;		//this is for drawing only
			x_draw2 = x_centre - 100;
			y_draw1 = y_centre + 100;
			y_draw2 = y_centre - 100;

			s.val[0] = 255;
	

			CvPoint pt1 = {x_draw1,y_centre};
			CvPoint pt2 = {x_draw2,y_centre};
			CvPoint pt3 = {x_centre,y_draw1};
			CvPoint pt4 = {x_centre,y_draw2};

			cvLine(&ipl_img, pt1 , pt2, s, 1, 8,0);
			cvLine(&ipl_img, pt3 , pt4, s, 1, 8,0);

			s.val[0] = 0;
			s.val[1] = 255;
			s.val[2] = 0;

			cvLine(&ipl_hls, pt1 , pt2, s, 1, 8,0);
			cvLine(&ipl_hls, pt3 , pt4, s, 1, 8,0);

			img_out_ = cv::Mat (&ipl_img).clone ();
			img_thresh_hls2_ = cv::Mat (&ipl_hls).clone ();

			//printf("X: %d Y: %d Count: %d Yeahhhhhhhhhhhhhhhhhhhhhh buoy!\n",x_centre,y_centre,count);
			ROS_DEBUG("Target Found at %d %d with a count of %d",x_centre,y_centre,count);
		}

		return;
	}

	void imageCallback (const sensor_msgs::ImageConstPtr & msg_ptr){
		// Convert ROS Imput Image Message to IplImage
		try{
			cv_input_ = bridge_.imgMsgToCv (msg_ptr, "bgr8");
		}
		catch(sensor_msgs::CvBridgeException error){
			ROS_ERROR ("CvBridge Input Error");
		}

		// Convert IplImage to cv::Mat
		img_in_ = cv::Mat (cv_input_).clone ();
		// Convert Input image from BGR to HSV
		cv::cvtColor (img_in_, img_hls_, CV_BGR2HLS);

		//cv::threshold(img_hls_, img_thresh_hls_, 120, 255, CV_THRESH_BINARY);
		//cv::threshold(img_hsv_, img_thresh_hsv_, 120, 255, CV_THRESH_BINARY);

		img_thresh_hls_ = img_hls_ > 80;
		img_thresh_hls2_ = img_thresh_hls_ < 70;

		cv::cvtColor (img_thresh_hls2_, img_grey_, CV_BGR2GRAY);

		img_out_ = img_grey_ > 70;

		findCentre();

		// Display Input image
		/*cv::imshow ("input", img_in_);
		cv::imshow ("hls", img_hls_);
		cv::imshow ("thresh hls up", img_thresh_hls_);
		cv::imshow ("thresh hls down", img_thresh_hls2_);
		cv::imshow ("grey", img_grey_);
		cv::imshow ("out", img_out_);*/
		//cv::imshow ("thresh hsv", img_thresh_hsv_);

		// Needed to  keep the HighGUI window open
		cv::waitKey (3);
	}

};


int main(int argc, char **argv){
	// Initialize ROS Node
	ros::init (argc, argv, "camtest");
	// Start node and create a Node Handle
	ros::NodeHandle nh;
	ros::Publisher fwdcamXMsg = nh.advertise<std_msgs::UInt32>("fwdcamX", 100);
	ros::Publisher fwdcamYMsg = nh.advertise<std_msgs::UInt32>("fwdcamY", 100);

	std_msgs::UInt32 fwdcamX;
	std_msgs::UInt32 fwdcamY;

	// Instaniate Demo Object
	ROS_INFO("Online");
	Demo d (nh);
	// Spin ...
	while(ros::ok()){
		ros::spinOnce();
		fwdcamX.data = x_centre;
		fwdcamY.data = y_centre;
		fwdcamXMsg.publish(fwdcamX);
		fwdcamYMsg.publish(fwdcamY);
	}
	// ... until done
	return 0;
}
