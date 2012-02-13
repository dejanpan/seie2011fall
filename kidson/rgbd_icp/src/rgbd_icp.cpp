/*
 * rgbd.cpp
 *
 *  Created on: Feb 13, 2012
 *      Author: ross
 */

#include "rgbd_icp.h"



void rgbd_icp::processRGBD_ICP(const rgbdslam::featureMatch& msg)
{
	//ROS_INFO("translation %f %f %f", msg.featureTransform.translation.x, msg.featureTransform.translation.y, msg.featureTransform.translation.z);
	std::vector<rgbdslam::match> local_matches = msg.matches;
  	for(std::vector<rgbdslam::match>::iterator iterator_ = local_matches.begin(); iterator_ != local_matches.end(); ++iterator_) {
  		//ROS_INFO("qidx: %d tidx: %d iidx: %d dist: %f", iterator_->queryId, iterator_->trainId, iterator_->imgId, iterator_->distance);
  	}

  	graphnode newNode;
  	newNode.id = 1;
  	newNode.pointCloud = msg.targetPointcloud;

  	//add node to datastrct
}

