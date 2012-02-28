#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
class ImageConverter {

public:
	ImageConverter(ros::NodeHandle &n) :
		n_(n), it_(n_)
	{
		cvNamedWindow("Image window");
		image_sub_ = it_.subscribe(
				"/camera/image_raw", 1, &ImageConverter::imageCallback, this);
		n_.param ("save_image", save_image_, std::string(""));
	}

	~ImageConverter()
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


		// Initialize capturing live feed from the camera


		// Couldn't get a device? Throw an error and quit
		if(!cv_image)
		{
			printf("Could not initialize capturing...\n");
			return ;
		}

		// The two windows we'll be using
		cvNamedWindow("video");
		cvNamedWindow("thresh yellow");
		cvNamedWindow("thresh green");



		// This image holds the "scribble" data...
		// the tracked positions of the ball
		IplImage* imgScribble = NULL;

		// An infinite loop
		while(true)
		{
			// Will hold a cv_image cv_imaged from the camera


			// If we couldn't grab a cv_image... quit
			if(!cv_image)
				return ;

			// If this is the first cv_image, we need to initialize it
			if(imgScribble == NULL)
			{
				imgScribble = cvCreateImage(cvGetSize(cv_image), 8, 3);
			}

			// Holds the yellow thresholded image (yellow = white, rest = black)
			IplImage* imgYellowThresh = GetThresholdedImage(cv_image,1);
			IplImage* imgGreenThresh = GetThresholdedImage(cv_image,2);

			// Calculate the moments to estimate the position of the ball
			CvMoments *moments_yellow = (CvMoments*)malloc(sizeof(CvMoments));
			CvMoments *moments_green = (CvMoments*)malloc(sizeof(CvMoments));


			cvMoments(imgYellowThresh, moments_yellow, 1);
			cvMoments(imgGreenThresh, moments_green, 1);


			// The actual moment values
			double moment10y = cvGetSpatialMoment(moments_yellow, 1, 0);
			double moment01y = cvGetSpatialMoment(moments_yellow, 0, 1);
			double areay = cvGetCentralMoment(moments_yellow, 0, 0);

			double moment10g = cvGetSpatialMoment(moments_green, 1, 0);
			double moment01g = cvGetSpatialMoment(moments_green, 0, 1);
			double areag = cvGetCentralMoment(moments_green, 0, 0);


			// Holding the last and current ball positions
			static int posXy = 0;
			static int posYy = 0;

			int lastXy = posXy;
			int lastYy = posYy;

			posXy = moment10y/areay;
			posYy = moment01y/areay;

			// Print it out for debugging purposes
			printf("position yellow (%d,%d)\n", posXy, posYy);



			static int posXg = 0;
			static int posYg = 0;

			int lastXg = posXg;
			int lastYg = posYg;

			posXg = moment10g/areag;
			posYg = moment01g/areag;

			// Print it out for debugging purposes
			printf("position green (%d,%d)\n", posXg, posYg);

			// We want to draw a line only if its a valid position
			if(lastXy>0 && lastYy>0 && posXy>0 && posYy>0)
			{
				// Draw a yellow line from the previous point to the current point
				cvLine(imgScribble, cvPoint(posXy, posYy), cvPoint(lastXy, lastYy), cvScalar(0,255,255), 5);
			}

			if(lastXg>0 && lastYg>0 && posXg>0 && posYg>0)
			{
				// Draw a yellow line from the previous point to the current point
				cvLine(imgScribble, cvPoint(posXg, posYg), cvPoint(lastXg, lastYg), cvScalar(0,255,0), 5);
			}



			// Add the scribbling image and the cv_image... and we get a combination of the two
			cvAdd(cv_image, imgScribble, cv_image);
			cvShowImage("thresh yellow", imgYellowThresh);
			cvShowImage("thresh green", imgGreenThresh);

			cvShowImage("video", cv_image);

			// Wait for a keypress
			int c = cvWaitKey(10);
			if(c!=-1)
			{
				// If pressed, break out of the loop
				return ;
			}

			// Release the thresholded image... we need no memory leaks.. please
			cvReleaseImage(&imgYellowThresh);
			cvReleaseImage(&imgGreenThresh);



			delete moments_yellow;
			delete moments_green;



			// end
			/* if (cv_image->width > 60 && cv_image->height > 60)
      cvCircle(cv_image, cvPoint(50,50), 10, cvScalar(255));

    cvShowImage("Image window", cv_image);
    n_.getParam("save_image", save_image_);
    if(save_image_ != "")
      {
        ROS_INFO("Saving image to %s", save_image_.c_str());
        cvSaveImage(save_image_.c_str(), cv_image);
      }
    cvWaitKey(3);*/
		}
	}

	IplImage* GetThresholdedImage(IplImage* img, int color)
		{
		// Convert the image into an HSV image
		IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
		cvCvtColor(img, imgHSV, CV_BGR2HSV);

		IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);
		if (color==1)
			// Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
			cvInRangeS(imgHSV, cvScalar(20, 100, 100), cvScalar(30, 255, 255), imgThreshed);

		if (color==2)
			// Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
			cvInRangeS(imgHSV, cvScalar(80, 100, 100), cvScalar(90, 255, 255), imgThreshed);

		if (color==3)
			// Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
			cvInRangeS(imgHSV, cvScalar(80, 100, 100), cvScalar(90, 255, 255), imgThreshed);

		if (color==4)
			// Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
			cvInRangeS(imgHSV, cvScalar(80, 100, 100), cvScalar(90, 255, 255), imgThreshed);

		cvReleaseImage(&imgHSV);

		return imgThreshed;

		}
protected:

	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	sensor_msgs::CvBridge bridge_;
	std::string save_image_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ros_to_openCv");
	ros::NodeHandle n("~");
	ImageConverter ic(n);
	ros::spin();
	return 0;
}
