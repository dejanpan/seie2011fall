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
#include <pcl/registration/transformation_estimation_lm.h>

Eigen::Matrix4f performJointOptimization (PointCloudConstPtr source_cloud_ptr,
    PointCloudConstPtr target_cloud_ptr, std::vector<int>& source_indices,
    std::vector<int>& target_indices, Eigen::Matrix4f& initial_transformation)
{
  //calculate normals for pointclouds
  ROS_INFO("Calculating normals...");
  PointCloudNormalsPtr source_normals_cloud_ptr (new PointCloudNormals);
  PointCloudNormalsPtr target_normals_cloud_ptr (new PointCloudNormals);
  calculatePointCloudNormals (source_cloud_ptr, source_normals_cloud_ptr);
  calculatePointCloudNormals (target_cloud_ptr, target_normals_cloud_ptr);

  boost::shared_ptr<TransformationEstimationWDF<PointNormal,PointNormal> >
    initial_transform_WDF (new TransformationEstimationWDF<PointNormal,PointNormal>());

  initial_transform_WDF->setAlpha (0.5);
  initial_transform_WDF->setCorrespondecesDFP (source_indices, target_indices);

//  // Instantiate ICP
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

  icp_wdf.setInputCloud (source_normals_cloud_ptr);
  icp_wdf.setInputTarget (target_normals_cloud_ptr);

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
