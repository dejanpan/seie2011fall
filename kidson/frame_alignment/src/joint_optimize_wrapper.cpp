/*
 * joint_optimize_wrapper.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#include "frame_alignment/joint_optimize_wrapper.h"
#include "frame_alignment/transformation_estimation_wdf.h"
#include "frame_alignment/pcl_utils.h"
#include "frame_alignment/parameter_server.h"

#include <ros/console.h>

#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

void checkforNaNs (const PointCloudNormalsConstPtr input_cloud_ptr)
{
  for (uint i = 0; i < input_cloud_ptr->points.size (); i++)
    if ( (input_cloud_ptr->points[i].x != input_cloud_ptr->points[i].x)
        || (input_cloud_ptr->points[i].y != input_cloud_ptr->points[i].y)
        || (input_cloud_ptr->points[i].z != input_cloud_ptr->points[i].z)
        || (input_cloud_ptr->points[i].normal_x != input_cloud_ptr->points[i].normal_x)
        || (input_cloud_ptr->points[i].normal_y != input_cloud_ptr->points[i].normal_y)
        || (input_cloud_ptr->points[i].normal_z != input_cloud_ptr->points[i].normal_z))
    {
      ROS_ERROR_STREAM("point has a NaN! idx" << i);
      ROS_ERROR_STREAM("x[" << input_cloud_ptr->points[i].x << "] y[" << input_cloud_ptr->points[i].y<< "] z[" << input_cloud_ptr->points[i].z<<
          "] xn[" << input_cloud_ptr->points[i].normal_x<< "] yn[" << input_cloud_ptr->points[i].normal_y<< "] zn[" << input_cloud_ptr->points[i].normal_z<< "]");
    }
}

Eigen::Matrix4f performJointOptimization (PointCloudConstPtr source_cloud_ptr,
    PointCloudConstPtr target_cloud_ptr, std::vector<Eigen::Vector4f>& source_feature_3d_locations,
    std::vector<Eigen::Vector4f>& target_feature_3d_locations,
    Eigen::Matrix4f& initial_transformation)
{
  //ICP cannot handle points with NaN values
  std::vector<int> removed_points;
  PointCloudPtr source_cloud_noNaN_ptr (new PointCloud);
  PointCloudPtr target_cloud_noNaN_ptr (new PointCloud);
  pcl::removeNaNFromPointCloud (*source_cloud_ptr, *source_cloud_noNaN_ptr, removed_points);
  pcl::removeNaNFromPointCloud (*target_cloud_ptr, *target_cloud_noNaN_ptr, removed_points);

  //pointcloud normals are required for icp point to plane
  ROS_INFO("Calculating normals...");
  PointCloudNormalsPtr source_cloud_normals_ptr (new PointCloudNormals);
  PointCloudNormalsPtr target_cloud_normals_ptr (new PointCloudNormals);
  calculatePointCloudNormals (source_cloud_noNaN_ptr, source_cloud_normals_ptr);
  calculatePointCloudNormals (target_cloud_noNaN_ptr, target_cloud_normals_ptr);

  checkforNaNs (source_cloud_normals_ptr);
  checkforNaNs (target_cloud_normals_ptr);

  // the indices of features are required by icp joint optimization
  std::vector<int> source_indices, target_indices;
  getIndicesFromMatches<PointNormal> (source_cloud_normals_ptr, source_feature_3d_locations,
      source_indices);
  getIndicesFromMatches<PointNormal> (target_cloud_normals_ptr, target_feature_3d_locations,
      target_indices);

  boost::shared_ptr<TransformationEstimationWDF<PointNormal, PointNormal> > initial_transform_WDF (
      new TransformationEstimationWDF<PointNormal, PointNormal> ());

  initial_transform_WDF->setAlpha (0.0);
  initial_transform_WDF->setCorrespondecesDFP (source_indices, target_indices);

  pcl::IterativeClosestPoint<PointNormal, PointNormal> icp_wdf;
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp_wdf.setMaxCorrespondenceDistance (0.05);
  // Set the maximum number of iterations (criterion 1)
  icp_wdf.setMaximumIterations (75);
  // Set the transformation epsilon (criterion 2)
  icp_wdf.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp_wdf.setEuclideanFitnessEpsilon (0);  //1
  // Set TransformationEstimationWDF as ICP transform estimator
  icp_wdf.setTransformationEstimation (initial_transform_WDF);

  icp_wdf.setInputCloud (source_cloud_normals_ptr);
  icp_wdf.setInputTarget (target_cloud_normals_ptr);

  PointCloudNormalsPtr cloud_transformed (new PointCloudNormals);
  ROS_INFO_STREAM(
      "---------------------------------------------------------      indices size: " << source_indices.size ());
  if (ParameterServer::instance ()->get<bool> ("use_ransac_to_initialize_icp"))
    icp_wdf.align (*cloud_transformed, initial_transformation);
  else
    icp_wdf.align (*cloud_transformed);

  ROS_INFO_STREAM(
      "[SIIMCloudMatch::runICPMatch] Has converged? = " << icp_wdf.hasConverged () << std::endl << " fitness score (SSD): " << icp_wdf.getFitnessScore (1000));

  return icp_wdf.getFinalTransformation ();
}
