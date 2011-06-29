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

// ROS/OpenCV HSV Demo
// Based on http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

int thresh = 0;

class Demo{

	protected:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		sensor_msgs::CvBridge bridge_;
		cv::Mat img_in_;
		cv::Mat img_hsv_;
		cv::Mat img_hue_;
		cv::Mat img_sat_;
		cv::Mat img_bin_;
		cv::Mat img_out_;
		cv::Mat img_out;
		cv::Mat img_bw;
		IplImage *cv_input_;

	public:

	Demo (ros::NodeHandle & nh):nh_ (nh), it_ (nh_){
		// Listen for image messages on a topic and setup callback
		image_sub_ = it_.subscribe ("/gscam/image_raw", 1, &Demo::imageCallback, this);
		// Open HighGUI Window
		cv::namedWindow ("input", 1);
		cv::namedWindow ("binary image", 1);
		cv::namedWindow ("segmented output", 1);
	}

	void findCentre(void){
		int x, y, count, x_estimate, y_estimate, x_centre, y_centre;
		int x_draw1, x_draw2, y_draw1, y_draw2;
		CvScalar s;

		x_estimate = y_estimate = count = 0;
	
		IplImage ipl_img = img_out;

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

		if(count < 10000){				//arbritrary size threshold
			printf("No Target in sight saw only %d\n",count);
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

			/*s.val[0] = 0;
			s.val[1] = 255;
			s.val[2] = 0;

			cvLine(g_gray, pt1 , pt2, s, 1, 8,0);
			cvLine(g_gray, pt3 , pt4, s, 1, 8,0);*/

			img_out = cv::Mat (&ipl_img).clone ();

			printf("X: %d Y: %d Count: %d Yeahhhhhhhhhhhhhhhhhhhhhh buoy!\n",x_centre,y_centre,count);
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
		// output = input
		img_out_ = img_in_.clone ();
		// Convert Input image from BGR to HSV
		cv::cvtColor (img_in_, img_hsv_, CV_BGR2HSV);
		//IplImage* img_bw = cvCreateImage(cvGetSize(img_hsv_),IPL_DEPTH_8U,1);
		img_bw = img_hsv_ > 60;
		img_bw = img_bw < 90;
		cv::cvtColor (img_bw, img_bin_, CV_BGR2GRAY);
		img_out = img_bin_ > 80;


		findCentre();

		// Display Input image
		cv::imshow ("input", img_in_);
		// Display Binary Image
		cv::imshow ("binary image", img_bw);
		// Display segmented image
		cv::imshow ("segmented output", img_out);

		// Needed to  keep the HighGUI window open
		cv::waitKey (3);
	}

};


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
