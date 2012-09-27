/*
 * main.cpp
 *
 *  Created on: 11.07.2012
 *      Author: ross Kidson
 */

#include <stdio.h>
#include <iostream>
#include <ros/ros.h>

//local files
#include "frame_alignment/typedefs.h"
#include "frame_alignment/RGB_feature_matcher.h"
#include "frame_alignment/pcl_utils.h"
#include "frame_alignment/joint_optimize_wrapper.h"

int main (int argc, char** argv)
{
  if (argc < 3)
  {
    ROS_WARN(
        "Please provide 2 .pcd files: rosrun frame_matcher joint_optimization source.pcd target.pcd");
    exit (0);
  }
  ros::init (argc, argv, "frame_alignment");
  if(!ros::master::check())
  {
    ROS_ERROR("roscore not running. stop.");
    exit(0);
  }
  PointCloudPtr source_cloud_ptr (new PointCloud);
  PointCloudPtr target_cloud_ptr (new PointCloud);
  pcl::PCDReader reader;
  reader.read (argv[1], *source_cloud_ptr);
  reader.read (argv[2], *target_cloud_ptr);

  // Extract 2d RGB features and project them into 3d.  Use Ransac to filter out outliers and
  // obtain a transformation between the 2 point clouds
  std::vector<Eigen::Vector4f> source_feature_3d_locations, target_feature_3d_locations;
  Eigen::Matrix4f ransac_trafo, joint_opt_trafo;
  RGBFeatureMatcher point_cloud_matcher (source_cloud_ptr, target_cloud_ptr);
  if (!point_cloud_matcher.getMatches (source_feature_3d_locations, target_feature_3d_locations,
      ransac_trafo))
    exit (0);

  // use the feature points as distinct correspondences in a joint optimization over dense
  // clouds and sparse feature points
  joint_opt_trafo = performJointOptimization (source_cloud_ptr, target_cloud_ptr,
      source_feature_3d_locations, target_feature_3d_locations, ransac_trafo);

  // write the resulting transformed pointcloud to disk
  transformAndWriteToFile (source_cloud_ptr, ransac_trafo);

  return 0;
}
