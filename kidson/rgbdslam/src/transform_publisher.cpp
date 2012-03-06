/*
 * transform_publisher.cpp
 *
 *  Created on: Feb 10, 2012
 *      Author: ross
 */
#include "transform_publisher.h"
#include "rgbdslam/featureMatch.h"
#include "rgbdslam/match.h"
#include "rgbdslam/featureLocation.h"

/** @brief Helper function to convert Eigen transformation to tf */
tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
    btMatrix3x3 btm;
    //ROS_INFO("trans: %f, %f, %f %f | %f, %f, %f %f | %f, %f, %f %f", trans(0,0), trans(0,1), trans(0,2), trans(0,3), trans(1,0), trans(1,1), trans(1,2), trans(1,3), trans(2,0), trans(2,1), trans(2,2), trans(2,3));
    btm.setValue(trans(0,0),trans(0,1),trans(0,2),
               trans(1,0),trans(1,1),trans(1,2),
               trans(2,0),trans(2,1),trans(2,2));
    btTransform ret;
    ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
    ret.setBasis(btm);
    return ret;
}

void publish_transform(MatchingResult mr, Node* targetPointCloud, Node* sourcePointCloud, ros::Publisher& featureMatchPub)
{

    //std_msgs::String msg;
	rgbdslam::featureMatch msg;

	//need to convert mr.final_trafo from Eigen::Matrix4f to geometry_msgs::Transform
  	geometry_msgs::Transform transform_msg;
  	tf::Transform trans;
  	trans = tfFromEigen(mr.final_trafo);
  	tf::transformTFToMsg(trans, transform_msg);
  	msg.featureTransform = transform_msg;
  	//ROS_INFO("translation %f, %f, %f", mr.final_trafo(0,3), mr.final_trafo(1,3), mr.final_trafo(2,3));

  	//convert pcl::PointCloud<pcl::PointXYZRGB> to sensor messages point cloud 2
  	sensor_msgs::PointCloud2 sourceCloudMessage;
  	sensor_msgs::PointCloud2 targetCloudMessage;
  	pcl::toROSMsg(*(sourcePointCloud->pc_col),sourceCloudMessage);
  	pcl::toROSMsg(*(targetPointCloud->pc_col),targetCloudMessage);
  	msg.sourcePointcloud = sourceCloudMessage;
  	msg.targetPointcloud = targetCloudMessage;

  	//extract matching result ids
  	for(std::vector<cv::DMatch>::iterator iterator_ = mr.inlier_matches.begin(); iterator_ != mr.inlier_matches.end(); ++iterator_) {
  	    rgbdslam::match matchmsg;
  	    matchmsg.queryId = iterator_->queryIdx;
  	    matchmsg.trainId = iterator_->trainIdx;
  	    matchmsg.imgId = iterator_->imgIdx;
  	    matchmsg.distance = iterator_->distance;
  	    msg.matches.push_back(matchmsg);
  	    //ROS_INFO("qidx: %d tidx: %d iidx: %d dist: %f", iterator_->queryIdx, iterator_->trainIdx, iterator_->imgIdx, iterator_->distance);
  	}

  	//Retrieve feature locations
  	for(std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >::iterator iterator_ = sourcePointCloud->feature_locations_3d_.begin(); iterator_ != sourcePointCloud->feature_locations_3d_.end(); ++iterator_) {
  		rgbdslam::featureLocation featureLocationMsg;
  		featureLocationMsg.x = iterator_->x();
  		featureLocationMsg.y = iterator_->y();
  		featureLocationMsg.z = iterator_->z();
  		featureLocationMsg.t = iterator_->w();
  		msg.sourceFeatureLocations.push_back(featureLocationMsg);
  	}
  	for(std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >::iterator iterator_ = targetPointCloud->feature_locations_3d_.begin(); iterator_ != targetPointCloud->feature_locations_3d_.end(); ++iterator_) {
  		rgbdslam::featureLocation featureLocationMsg;
  		featureLocationMsg.x = iterator_->x();
  		featureLocationMsg.y = iterator_->y();
  		featureLocationMsg.z = iterator_->z();
  		featureLocationMsg.t = iterator_->w();
  		msg.targetFeatureLocations.push_back(featureLocationMsg);
  	}

    featureMatchPub.publish(msg);
}
