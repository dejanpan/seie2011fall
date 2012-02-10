/*
 * transform_publisher.cpp
 *
 *  Created on: Feb 10, 2012
 *      Author: ross
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "node.h"
#include "matching_result.h"
#include "rgbdslam/featureMatch.h"
#include "rgbdslam/match.h"
#include "pcl_ros/transforms.h"

/** @brief Helper function to convert Eigen transformation to tf -- thanks to Garret Gallagher */
tf::Transform tfFromEigen(Eigen::Matrix4f trans);

void publish_transform(MatchingResult mr, Node* targetPointCloud, Node* sourcePointCloud, ros::Publisher& featureMatchPub);
