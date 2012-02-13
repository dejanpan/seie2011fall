/*
 * rgbd_icp.h
 *
 *  Created on: Feb 13, 2012
 *      Author: ross
 */

#ifndef RGBD_ICP_H_
#define RGBD_ICP_H_

#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "rgbdslam/featureMatch.h"
#include "rgbdslam/match.h"
#include "pcl_ros/transforms.h"
#include <opencv2/features2d/features2d.hpp>
#include <vector>

void processRGBD_ICP(const rgbdslam::featureMatch& msg);


#endif /* RGBD_ICP_H_ */
