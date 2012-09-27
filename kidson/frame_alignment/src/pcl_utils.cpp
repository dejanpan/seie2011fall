/*
 * pcl_utils.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#include "frame_alignment/pcl_utils.h"
#include "frame_alignment/parameter_server.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>

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

void calculatePointCloudNormals (const PointCloudConstPtr input_cloud_ptr,
    PointCloudNormalsPtr output_cloud_ptr)
{
  PointCloudPtr filtered (new PointCloud);
  pcl::NormalEstimation<PointType, PointNormal>::Ptr normal_estimator_ptr;
  if(ParameterServer::instance()->get<bool>("use_openmp_normal_calculation"))
    normal_estimator_ptr.reset(new pcl::NormalEstimationOMP<PointType, PointNormal>);
  else
    normal_estimator_ptr.reset (new pcl::NormalEstimation<PointType, PointNormal>);

//  std::vector<int> indices;
//  pcl::removeNaNFromPointCloud(*input_cloud_ptr, *filtered, indices);
  normal_estimator_ptr->setInputCloud (input_cloud_ptr);
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
  normal_estimator_ptr->setSearchMethod (tree);
  normal_estimator_ptr->setRadiusSearch (0.05);
  normal_estimator_ptr->compute (*output_cloud_ptr);
  pcl::copyPointCloud (*input_cloud_ptr, *output_cloud_ptr);
}
