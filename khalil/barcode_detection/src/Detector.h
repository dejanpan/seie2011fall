/*
 * detector.h
 *
 *  Created on: Mar 14, 2012
 *      Author: banacer
 */

#ifndef DETECTOR_H_
#define DETECTOR_H_
#include <opencv-2.3.1/opencv/cv.h>
#include <opencv-2.3.1/opencv/highgui.h>
#include <opencv-2.3.1/opencv/cxcore.h>
#include <opencv-2.3.1/opencv2/core/core_c.h>
#include <opencv-2.3.1/opencv2/core/core.hpp>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace std
{

	class Detector
	{
	public:
		Detector(ros::NodeHandle & , float , int , int , int );
	  virtual ~Detector();

	  float maxDistance;
	  int minTransitions;
	  int maxTransitions;
	  int minROIlength;
	  ros::Subscriber sub;
	  ros::Publisher publisher;
	  cv_bridge::CvImagePtr cv_bridge_ptr;
	  cv_bridge::CvImagePtr out_msg_ptr;
	  ros::NodeHandle nh_;

	  void ImageCallback(const sensor_msgs::ImageConstPtr& );
	  void SlidingwindowDetection(float , int , int , int );
	  void findSquare(double**  ,int , int ,int ,float ,int );
	};

} /* namespace std */
#endif /* DETECTOR_H_ */
