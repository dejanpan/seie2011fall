
#include "rgbd_icp.h"

void processRGBD_ICP(const rgbdslam::featureMatch& msg)
{
	//ROS_INFO("translation %f %f %f", msg.featureTransform.translation.x, msg.featureTransform.translation.y, msg.featureTransform.translation.z);
	std::vector<rgbdslam::match> local_matches = msg.matches;
  	for(std::vector<rgbdslam::match>::iterator iterator_ = local_matches.begin(); iterator_ != local_matches.end(); ++iterator_) {

  		//ROS_INFO("qidx: %d tidx: %d iidx: %d dist: %f", iterator_->queryId, iterator_->trainId, iterator_->imgId, iterator_->distance);

  	}
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/feature_match_out_topic", 1000, processRGBD_ICP);
  ros::spin();

  return 0;
}
