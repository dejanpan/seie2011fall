#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include "node.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/CvBridge.h>
#include "parameter_server.h"
#include <rosbag/view.h>
#include <boost/foreach.hpp>

class PointCloudCapturer
{
  ros::NodeHandle nh_;

  double rate_;
  rosbag::Bag bag_;
  std::string bag_name_;
  tf::TransformListener tf_;
  std::string to_frame_;

  std::string cloud_topic_, image_topic_, camera_info_topic_;
  bool cloud_and_image_received_, move_head_;
  sensor_msgs::CvBridge bridge;

  sensor_msgs::CameraInfoConstPtr cam_info_;
public:
  PointCloudCapturer()
  {
	  cloud_topic_ =  ParameterServer::instance()->get<std::string>("topic_points");
	  image_topic_ =  ParameterServer::instance()->get<std::string>("topic_image_mono");
	  camera_info_topic_ = ParameterServer::instance()->get<std::string>("camera_info_topic");
	  bag_name_ = "RGBD_Output.bag";
	  to_frame_ = "base_link";
	  rate_ = 1.0;
	  bag_.open(bag_name_, rosbag::bagmode::Write);

	  //get the camera info (just once)
	std::string bagfile_name = ParameterServer::instance()->get<std::string>("bagfile_name");
	if(bagfile_name.empty())
		cam_info_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_);
	else
	{
		rosbag::Bag bag;
		bag.open(bagfile_name, rosbag::bagmode::Read);

		std::vector<std::string> topics;
		topics.push_back(camera_info_topic_);

		rosbag::View view(bag, rosbag::TopicQuery(topics));

		BOOST_FOREACH(rosbag::MessageInstance const m, view)
		{
			 if ((m.getTopic() == camera_info_topic_) && (cam_info_ == NULL))
			 {
				 cam_info_ = m.instantiate<sensor_msgs::CameraInfo>();
				 ROS_INFO("cam info pointer set");
			 }
		}
	}
  }

  ~PointCloudCapturer()
  {
    bag_.close();
   // delete point_head_client_;
  }

  void saveCloudsToBagfile(Node* node_, tf::Transform nodeTransform){
  	ros::Time now = ros::Time::now(); //makes sure things have a corresponding timestamp

  	/***********Write data to a bag file ******************/
  	// todo:move to a function

  	//Writing cloud to bagfile
  	sensor_msgs::PointCloud2 cloudMessage;
  	node_->reloadPointCloudFromDisk();
  	pcl::toROSMsg(*(node_->pc_col),cloudMessage);
  	cloudMessage.header.frame_id = "/openni_rgb_optical_frame"; //?????
  	cloudMessage.header.stamp = now;
  	bag_.write(cloud_topic_, cloudMessage.header.stamp, cloudMessage);
  	ROS_INFO("Wrote cloud to %s", bag_name_.c_str());
  	node_->clearPointCloud();

  	//Writing pointcloud transform to bag file
  	geometry_msgs::Transform transform_msg;
  	tf::transformTFToMsg(nodeTransform, transform_msg);
  	bag_.write(cloud_topic_ + "/transform", cloudMessage.header.stamp, transform_msg);

  	//writing image transform to bag file
    //Get images into message format

  	//sensor_msgs::ImageConstPtr im = Mat2Image(node_->cameraImageColour);
  	cv::Mat aMat = node_->cameraImageColour;
  	ROS_INFO("made aMat");
    IplImage iplimg = aMat;
    ROS_INFO("converted aMat to ipl");
    sensor_msgs::ImageConstPtr im =  bridge.cvToImgMsg(&iplimg, "passthrough");
    ROS_INFO("converted mat to rosmsg");
    //im->header.frame_id = "/openni_rgb_optical_frame"; //?????
  	bag_.write(image_topic_, cloudMessage.header.stamp, im);
  	ROS_INFO("Wrote image to %s", bag_name_.c_str());
  	bag_.write(image_topic_ + "/transform", cloudMessage.header.stamp, transform_msg);

  	bag_.write(camera_info_topic_, cam_info_->header.stamp, cam_info_);
  	ROS_INFO("Wrote Camera Info to %s", bag_name_.c_str());
  }
};
