#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//boost
#include <boost/thread/mutex.hpp>

//for moving of the head
//#include <actionlib/client/simple_action_client.h>
//#include <pr2_controllers_msgs/PointHeadAction.h>

//for writting to bag
#include <rosbag/bag.h>

//msg synchronisation
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include "pcl_ros/transforms.h"
#include <tf/transform_listener.h>

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
// Our Action interface type, provided as a typedef for convenience
//typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
//typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2, sensor_msgs::Image > MySyncPolicy;

class PointCloudCapturer
{
  ros::NodeHandle nh_;

  double rate_;
  rosbag::Bag bag_;
  std::string bag_name_;
  tf::TransformListener tf_;
  std::string to_frame_;
  //ros::Subscriber camera_sub_;

  std::string input_cloud_topic_, input_image_topic_, input_camera_info_topic_;
  bool cloud_and_image_received_, move_head_;

  //message_filters::Subscriber<sensor_msgs::Image> camera_sub_;
  //message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  //message_filters::Synchronizer<MySyncPolicy> synchronizer_;
  //message_filters::Connection sync_connection_;
  sensor_msgs::CameraInfoConstPtr cam_info_;
public:
  PointCloudCapturer() //ros::NodeHandle &n
	//:  nh_(n)
  {
    /*nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/camera/rgb/points"));
    nh_.param("input_image_topic", input_image_topic_, std::string("/camera/rgb/image_color"));
    nh_.param("input_camera_info_topic", input_camera_info_topic_, std::string("/camera/rgb/camera_info"));

    nh_.param("bag_name", bag_name_, std::string("bosch_kitchen_tr.bag"));
    nh_.param("to_frame", to_frame_, std::string("base_link"));
    nh_.param("rate", rate_, 1.0);*/

	  input_cloud_topic_ = "/camera/rgb/points";
	  input_image_topic_ = "/camera/rgb/image_color";
	  input_camera_info_topic_ = "/camera/rgb/camera_info";
	  bag_name_ = "bosch_kitchen_tr.bag";
	  to_frame_ = "base_link";
	  rate_ = 1.0;

	  bag_.open(bag_name_, rosbag::bagmode::Write);
  }

  ~PointCloudCapturer()
  {
    bag_.close();
   // delete point_head_client_;
  }

//  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  void callback (const sensor_msgs::PointCloud2ConstPtr& pc, const sensor_msgs::ImageConstPtr& im)
  {
      //Write cloud
      //transform to target frame
      bool found_transform = tf_.waitForTransform(pc->header.frame_id, to_frame_,
                                                  pc->header.stamp, ros::Duration(10.0));
      tf::StampedTransform transform;
      if (found_transform)
      {
        //ROS_ASSERT_MSG(found_transform, "Could not transform to camera frame");
        tf_.lookupTransform(to_frame_,pc->header.frame_id, pc->header.stamp, transform);
        ROS_DEBUG("[TransformPointcloudNode:] Point cloud published in frame %s", pc->header.frame_id.c_str());
      }
      else
      {
        ROS_ERROR("No transform for pointcloud found!!!");
        return;
      }
      bag_.write(input_cloud_topic_, pc->header.stamp, *pc);
      geometry_msgs::TransformStamped transform_msg;
      tf::transformStampedTFToMsg(transform, transform_msg);
      bag_.write(input_cloud_topic_ + "/transform", transform_msg.header.stamp, transform_msg);
      ROS_INFO("Wrote cloud to %s", bag_name_.c_str());

      //Write image
      found_transform = tf_.waitForTransform(im->header.frame_id, to_frame_,
                                                  im->header.stamp, ros::Duration(10.0));
      if (found_transform)
      {
        //ROS_ASSERT_MSG(found_transform, "Could not transform to camera frame");
        tf_.lookupTransform(to_frame_,im->header.frame_id, im->header.stamp, transform);
        ROS_DEBUG("[TransformPointcloudNode:] Point cloud published in frame %s", im->header.frame_id.c_str());
      }
      else
      {
        ROS_ERROR("No transform for image found!!!");
        return;
      }
      bag_.write(input_image_topic_, im->header.stamp, im);
      tf::transformStampedTFToMsg(transform, transform_msg);
      bag_.write(input_image_topic_ + "/transform", transform_msg.header.stamp, transform_msg);
      ROS_INFO("Wrote image to %s", bag_name_.c_str());

      cam_info_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(input_camera_info_topic_);
      bag_.write(input_camera_info_topic_, cam_info_->header.stamp, cam_info_);
      ROS_INFO("Wrote Camera Info to %s", bag_name_.c_str());
      cloud_and_image_received_ = true;
  }
};
