#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
class ImageSubscriber {

public:
	ImageSubscriber(ros::NodeHandle &n) :
		n_(n), it_(n_)
	{
		cvNamedWindow("Image window");
		image_sub_ = it_.subscribe(
				"/camera/image_raw", 1, &ImageSubscriber::imageCallback, this);
		n_.param ("save_image", save_image_, std::string(""));
	}

	~ImageSubscriber()
	{
		cvDestroyWindow("Image window");
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
	{

		IplImage *cv_image = NULL;
		try
		{
			cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("error");
		}


		// If we couldn't grab a cv_image... quit
		if(!cv_image)
			return ;

		if (imgScribble == NULL) {
			imgScribble = cvCreateImage(cvGetSize(cv_image), 8, 3);
		}


}

void drawMatchesOnly(cv::Mat& image1, const std::vector<cv::KeyPoint>& keypoints1,
			cv::Mat& image2, const std::vector<cv::KeyPoint>& keypoints2,
			const std::vector<f2d::Match>& matches, cv::Mat& display)
	{
		// Set up composite image
		//display = cv::Mat::zeros(std::max(image1.rows, image2.rows), image1.cols + image2.cols, CV_8UC3);
		//cv::Mat sub_display1 = display( cv::Rect(0, 0, image1.cols, image1.rows) );
		//cv::Mat sub_display2 = display( cv::Rect(image1.cols, 0, image2.cols, image2.rows) );

		//cv::resize(image1,image1, Size(512,384),0,0,INTER_LINEAR);
		//cv::resize(image2,image2,Size(512,384),0,0,INTER_LINEAR);

		cvtColor(image1,image1,CV_GRAY2BGR);
		cvtColor(image2,image2,CV_GRAY2BGR);


		// Draw lines between matches
		//int shift_bits = 4; //check what they are doing
		//int multiplier = 1 << shift_bits; //check
		printf("Size of the matches: %d\n",matches.size());
		for (std::vector<f2d::Match>::const_iterator i = matches.begin(), ie = matches.end(); i != ie; ++i) {
			const cv::KeyPoint& keypt1 = keypoints1[i->index1];
			const cv::KeyPoint& keypt2 = keypoints2[i->index2];
			const Scalar red =   CV_RGB(255, 0, 0);
			const Scalar color = CV_RGB(std::rand() % 256, std::rand() % 256, std::rand() % 256);
			const int thickness=8,lineType=8,radius=2;
			cv::circle(image1,Point(keypt1.pt.x,keypt1.pt.y),radius,color,thickness,lineType);
			printf("Keypoint1 x : %f, y : %f\n",keypt1.pt.x,keypt1.pt.y);
			printf("Keypoint2 x : %f, y : %f\n",keypt2.pt.x,keypt2.pt.y);
			cv::circle(image2,Point(keypt2.pt.x,keypt2.pt.y),radius,color,thickness,lineType);
			//cv::line(display,Point(keypt1.pt.x,keypt1.pt.y),Point(keypt2.pt.x,keypt2.pt.y),red, 1, CV_AA, shift_bits);

			//cvShowImage("Image window 2",image2);

			//cv::Point center1(keypt1.pt.x * multiplier, keypt1.pt.y * multiplier); //try resizing image 640*480
			//cv::Point center2((keypt2.pt.x + image1.cols) * multiplier, keypt2.pt.y * multiplier);
			//cv::Scalar color(std::rand() % 256, std::rand() % 256, std::rand() % 256);
			//cv::line(display, center1, center2, color, 1, CV_AA, shift_bits);
		}
		imshow("Window1",image1);
		imshow("Window2",image2);
		waitKey(0);
	}

}
