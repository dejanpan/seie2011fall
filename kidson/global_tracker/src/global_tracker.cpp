/*
 * global_tracker.cpp
 *
 *  Created on: Sep 1, 2011
 *      Author: engelhar
 */

#include "global_tracker.h"


 GlobalTracker::GlobalTracker(unsigned int priority, std::string *_sensorFrame,
                              cv::Mat *image,
                              cv::Mat *depthImage,
                              image_geometry::PinholeCameraModel *cam)
 {
   rgb_img = image;
 }



bool GlobalTracker::processFrame(g2o::VertexSE3 *frameVertex){

  cv::namedWindow("global tracker");
  cv::imshow("global tracker", *rgb_img);
  cv::waitKey(10);

}
