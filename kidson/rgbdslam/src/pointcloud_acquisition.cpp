#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include "node.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/Image.h"

class PointCloudCapturer
{
  ros::NodeHandle nh_;

  double rate_;
  rosbag::Bag bag_;
  std::string bag_name_;
  tf::TransformListener tf_;
  std::string to_frame_;

  std::string input_cloud_topic_, input_image_topic_, input_camera_info_topic_;
  bool cloud_and_image_received_, move_head_;

  sensor_msgs::CameraInfoConstPtr cam_info_;
public:
  PointCloudCapturer()
  {
	  input_cloud_topic_ = "/camera/rgb/points";
	  input_image_topic_ = "/camera/rgb/image_color";
	  input_camera_info_topic_ = "/camera/rgb/camera_info";
	  bag_name_ = "RGBD_Output.bag";
	  to_frame_ = "base_link";
	  rate_ = 1.0;
	  bag_.open(bag_name_, rosbag::bagmode::Write);
  }

  ~PointCloudCapturer()
  {
    bag_.close();
   // delete point_head_client_;
  }

  void saveCloudsToBagfile(Node* node_, tf::Transform nodeTransform){
  	sensor_msgs::CameraInfoConstPtr cam_info_;
  	std::string bag_name_ = "RecordedGraph.bag";
  	std::string cloud_topic_ = "/camera/rgb/points";
  	std::string image_topic_ = "/camera/rgb/image_color";
  	std::string camera_info_topic_ = "/camera/rgb/camera_info";
  	ros::Time now = ros::Time::now(); //makes sure things have a corresponding timestamp

  	/***********Write data to a bag file ******************/
  	// todo:move to a function

  	//Writing cloud to bagfile
  	sensor_msgs::PointCloud2 cloudMessage;
  	pcl::toROSMsg(*(node_->pc_col),cloudMessage);
  	cloudMessage.header.frame_id = "/openni_rgb_optical_frame"; //?????
  	cloudMessage.header.stamp = now;
  	bag_.write(cloud_topic_, cloudMessage.header.stamp, cloudMessage);
  	ROS_INFO("Wrote cloud to %s", bag_name_.c_str());

  	//Writing pointcloud transform to bag file
  	geometry_msgs::Transform transform_msg;
  	tf::transformTFToMsg(nodeTransform, transform_msg);
  	bag_.write(cloud_topic_ + "/transform", cloudMessage.header.stamp, transform_msg);

  	//writing image transform to bag file
  	//bag_.write(image_topic_ + "/transform", transform_msg.header.stamp, transform_msg);
  	//bag_.write(image_topic_, im->header.stamp, im);
  	//ROS_INFO("Wrote image to %s", bag_name_.c_str());

  	//writing camera image to bag file
  	cam_info_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_);
  	bag_.write(camera_info_topic_, cam_info_->header.stamp, cam_info_);
  	ROS_INFO("Wrote Camera Info to %s", bag_name_.c_str());
  }
};
