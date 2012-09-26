/*
 * pcl_utils.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#include "frame_alignment/typedefs.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

void transformAndWriteToFile (const PointCloudConstPtr cloud_in, const Eigen::Matrix4f& trafo)
{
  PointCloudPtr cloud_out (new PointCloud);
  pcl::transformPointCloud (*cloud_in, *cloud_out, trafo);
  pcl::PCDWriter writer;
  writer.write ("transformed_cloud.pcd", *cloud_out, false);
}

void writeFeaturePointCloudsToFile (const PointCloudConstPtr source_cloud,
    const std::vector<int>& source_indices, const PointCloudConstPtr target_cloud,
    const std::vector<int> target_indices, const Eigen::Matrix4f& trafo)
{
  PointCloudPtr transformed_cloud (new PointCloud);
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, trafo);

  pcl::PCDWriter writer;
  writer.write ("source_feature_cloud.pcd", *source_cloud, source_indices, false);
  writer.write ("source_cloud.pcd", *source_cloud, false);
  writer.write ("target_feature_cloud.pcd", *target_cloud, target_indices, false);
  writer.write ("transformed_feature_cloud.pcd", *transformed_cloud, source_indices, false);
}
