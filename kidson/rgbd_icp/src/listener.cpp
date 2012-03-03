/*
 * listener.cpp
 *
 *  Created on: Feb 13, 2012
 *      Author: ross
 */

#include "ros/ros.h"
#include "rgbd_icp.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  rgbd_icp rgbd_listener;
  ros::Subscriber sub = n.subscribe("/feature_match_out_topic", 1000, &rgbd_icp::processRGBD_ICP, &rgbd_listener);
  ros::spin();
  return 0;
}
