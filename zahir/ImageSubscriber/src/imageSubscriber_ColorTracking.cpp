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

		// Holds the color thresholded image (color = white, rest = black)
		IplImage* imgYellowThresh = GetThresholdedImage(cv_image, 4);
		IplImage* imgGreenThresh = GetThresholdedImage(cv_image, 1);
		IplImage* imgBlueThresh = GetThresholdedImage(cv_image, 2);
		IplImage* imgRedThresh = GetThresholdedImage(cv_image, 3);

		// Calculate the moments to estimate the position of the ball
		CvMoments *moments_yellow = (CvMoments*) malloc(sizeof(CvMoments));
		CvMoments *moments_green = (CvMoments*) malloc(sizeof(CvMoments));
		CvMoments *moments_blue = (CvMoments*) malloc(sizeof(CvMoments));
		CvMoments *moments_red = (CvMoments*) malloc(sizeof(CvMoments));

		cvMoments(imgYellowThresh, moments_yellow, 1);
		cvMoments(imgGreenThresh, moments_green, 1);
		cvMoments(imgBlueThresh, moments_blue, 1);
		cvMoments(imgRedThresh, moments_red, 1);

		// The actual moment values
		double moment10y = cvGetSpatialMoment(moments_yellow, 1, 0);
		double moment01y = cvGetSpatialMoment(moments_yellow, 0, 1);
		double areay = cvGetCentralMoment(moments_yellow, 0, 0);

		double moment10g = cvGetSpatialMoment(moments_green, 1, 0);
		double moment01g = cvGetSpatialMoment(moments_green, 0, 1);
		double areag = cvGetCentralMoment(moments_green, 0, 0);

		double moment10b = cvGetSpatialMoment(moments_blue, 1, 0);
		double moment01b = cvGetSpatialMoment(moments_blue, 0, 1);
		double areab = cvGetCentralMoment(moments_blue, 0, 0);

		double moment10r = cvGetSpatialMoment(moments_red, 1, 0);
		double moment01r = cvGetSpatialMoment(moments_red, 0, 1);
		double arear = cvGetCentralMoment(moments_red, 0, 0);

		//* Yellow processing    *//
		// Holding the last and current color positions
		static int posXy = 0;
		static int posYy = 0;

		lastXy = posXy;
		lastYy = posYy;

		int tempXy = moment10y / areay;
		int tempYy = moment01y / areay;

		if ( tempXy >= 0 && tempYy >= 0  && tempXy < 700 && tempYy < 700 && areay>1000 ){
			posXy = moment10y / areay;
			posYy = moment01y / areay;
		}



		// Print it out for debugging purposes
		std::cout << std::endl;
		printf("position yellow (%d,%d)\n", posXy, posYy);
		std::cout  <<"area yellow : "<< areay  << std::endl;

		//* Green Processing    *//
		static int posXg = 0;
		static int posYg = 0;

		lastXg = posXg;
		lastYg = posYg;

		int tempXg = moment10g / areag;
		int tempYg = moment01g / areag;

		if ( tempXg >= 0 && tempYg >= 0  && tempXg < 700 && tempYg < 700 && areag>1000 ){
			posXg = moment10g / areag;
			posYg = moment01g / areag;
		}



		// Print it out for debugging purposes
		printf("position green (%d,%d)\n", posXg, posYg);
		std::cout  <<"area green : "<< areag  << std::endl;


		static int posXb = 0;
		static int posYb = 0;

		lastXb = posXb;
		lastYb = posYb;

		int tempXb = moment10b / areab;
		int tempYb = moment01b / areab;

		if ( tempXb >= 0 && tempYb >= 0  && tempXb < 700 && tempYb < 700 && areab>700 ){
			posXb = moment10b / areab;
			posYb = moment01b / areab;
		}


		// Print it out for debugging purposes
		printf("position blue (%d,%d)\n", posXb, posYb);
		std::cout  <<"area blue : "<< areab  << std::endl;


		static int posXr = 0;
		static int posYr = 0;

		lastXr = posXr;
		lastYr = posYr;

		int tempXr = moment10r / arear;
		int tempYr = moment01r / arear;

		if ( tempXr >= 0 && tempYr >= 0  && tempXr < 700 && tempYr < 700 && arear>1000 ){
			posXr = moment10r / arear;
			posYr = moment01r / arear;
		}


		printf("position red (%d,%d)\n", posXr, posYr);
		std::cout  <<"area red : "<< arear  << std::endl;




		// We want to draw a line only if its a valid position
		if (lastXy > 0 && lastYy > 0 && posXy > 0 && posYy > 0 && lastXy < 700 && lastYy < 700 && posXy < 700 && posYy < 700) {
			// Draw a line from the previous point to the current point
			cvLine(imgScribble, cvPoint(posXy, posYy), cvPoint(lastXy,
					lastYy), cvScalar(0, 255, 255), 5);
		}
        //never enters here
		if (lastXg > 0 && lastYg > 0 && posXg > 0 && posYg > 0 && lastXg < 700 && lastYg < 700 && posXg < 700 && posYg < 700) {
			cvLine(imgScribble, cvPoint(posXg, posYg), cvPoint(lastXg,
					lastYg), cvScalar(0, 255, 0), 5);
		}

		// never enters here
		if (lastXb > 0 && lastYb > 0 && posXb > 0 && posYb > 0&& lastXb < 700 && lastYb < 700 && posXb < 700 && posYb < 700) {
			cvLine(imgScribble, cvPoint(posXb, posYb), cvPoint(lastXb,
					lastYb), cvScalar(255, 0, 0), 5);
		}

		if (lastXr > 0 && lastYr > 0 && posXr > 0 && posYr > 0 && lastXr < 700 && lastYr < 700 && posXr < 700 && posYr < 700) {
			cvLine(imgScribble, cvPoint(posXr, posYr), cvPoint(lastXr,
					lastYr), cvScalar(0, 0, 255), 5);
		}

		// Add the scribbling image and the cv_image... and we get a combination of the 4
		cvAdd(cv_image, imgScribble, cv_image);
		cvShowImage("thresh yellow", imgYellowThresh);
		cvShowImage("thresh green", imgGreenThresh);
		cvShowImage("thresh blue", imgBlueThresh);
		cvShowImage("thresh red", imgRedThresh);
		cvShowImage("video", cv_image);

		int c = cvWaitKey(10);
		if(c!=-1)
		{
			// If pressed, break out of the loop
			return ;
		}

		// Release the thresholded image
		cvReleaseImage(&imgYellowThresh);
		cvReleaseImage(&imgGreenThresh);
		cvReleaseImage(&imgBlueThresh);
		cvReleaseImage(&imgRedThresh);

		delete moments_yellow;
		delete moments_green;
		delete moments_blue;
		delete moments_red;

		//	cvShowImage("video", cv_image);

	}


	IplImage* GetThresholdedImage(IplImage* img, int color) {
		// Convert the image into an HSV image
		IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
		cvCvtColor(img, imgHSV, CV_BGR2HSV);

		IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);
		if (color == 1)
			//    cvInRangeS(imgHSV, cvScalar(10, 100, 100), cvScalar(20, 255, 255), imgThreshed);- orange paper
			//    cvInRangeS(imgHSV, cvScalar(155, 100, 100), cvScalar(165, 255, 255), imgThreshed); - pink one
			//    cvInRangeS(imgHSV, cvScalar(80, 100, 100), cvScalar(90, 255, 255), imgThreshed); - green robot
			//    cvInRangeS(imgHSV, cvScalar(40, 100, 100), cvScalar(50, 255, 255), imgThreshed);  - green paper
			//    cvInRangeS(imgHSV, cvScalar(100, 100, 100), cvScalar(110, 255, 255), imgThreshed); - blue tape


			cvInRangeS(imgHSV, cvScalar(40, 100, 100), cvScalar(50, 255, 255),
					imgThreshed);

		if (color == 2)
			cvInRangeS(imgHSV, cvScalar(100, 100, 100), cvScalar(110, 255, 255),
					imgThreshed);

		if (color == 3)

			cvInRangeS(imgHSV, cvScalar(155, 100, 100), cvScalar(165, 255, 255),
					imgThreshed);

		if (color == 4)
			cvInRangeS(imgHSV, cvScalar(10, 100, 100), cvScalar(20, 255, 255),
					imgThreshed);

		cvReleaseImage(&imgHSV);

		return imgThreshed;
	}

protected:

	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	sensor_msgs::CvBridge bridge_;
	std::string save_image_;
	IplImage* imgScribble;
	int lastXy;
	int lastYy;
	int lastXg;
	int lastYg;
	int lastYb;
	int lastXb;
	int lastYr;
	int lastXr;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rosToOpenCv");
	ros::NodeHandle n("~");
	ImageSubscriber ic(n);
	ros::spin();
	return 0;
}
