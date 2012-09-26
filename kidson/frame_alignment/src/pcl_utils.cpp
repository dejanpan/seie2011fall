/*
 * pcl_utils.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#include "frame_alignment/pcl_utils.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <ros/console.h>

void transformAndWriteToFile (const PointCloudConstPtr cloud_in, const Eigen::Matrix4f& trafo)
{
  PointCloudPtr cloud_out (new PointCloud);
  pcl::transformPointCloud (*cloud_in, *cloud_out, trafo);
  writePCDToFile ("transformed_cloud.pcd", cloud_out);
}

void writeFeaturePointCloudsToFile (const PointCloudConstPtr source_cloud,
    const std::vector<int>& source_indices, const PointCloudConstPtr target_cloud,
    const std::vector<int> target_indices, const Eigen::Matrix4f& trafo)
{
  PointCloudPtr transformed_cloud (new PointCloud);
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, trafo);

  writePCDToFile ("source_feature_cloud.pcd", source_cloud, source_indices);
  writePCDToFile ("source_cloud.pcd", source_cloud);
  writePCDToFile ("target_feature_cloud.pcd", target_cloud, target_indices);
  writePCDToFile ("target_cloud.pcd", target_cloud);
  writePCDToFile ("transformed_feature_cloud.pcd", transformed_cloud, source_indices);
}

void writePCDToFile (const std::string& fileName, const PointCloudConstPtr cloud_ptr)
{
  pcl::PCDWriter writer;
  ROS_INFO_STREAM("Writing point cloud " << fileName << " to file");
  writer.write (fileName, *cloud_ptr, false);
}

void writePCDToFile (const std::string& fileName, const PointCloudConstPtr cloud_ptr,
    const std::vector<int>& indices)
{
  pcl::PCDWriter writer;
  ROS_INFO_STREAM("Writing point cloud " << fileName << " to file");
  writer.write (fileName, *cloud_ptr, indices, false);
}
