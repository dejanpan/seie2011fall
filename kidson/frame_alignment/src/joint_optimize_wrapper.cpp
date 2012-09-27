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

  // the indices of features are required by icp joint optimization
  std::vector<int> source_indices, target_indices;
  getIndicesFromMatches<PointNormal> (source_cloud_normals_ptr, source_feature_3d_locations,
      source_indices);
  getIndicesFromMatches<PointNormal> (target_cloud_normals_ptr, target_feature_3d_locations,
      target_indices);


  boost::shared_ptr<TransformationEstimationWDF<PointNormal, PointNormal> > initial_transform_WDF (
      new TransformationEstimationWDF<PointNormal, PointNormal> ());

  // Please see parameter_server.cpp for an explanation of the following parameters
  ParameterServer* ps = ParameterServer::instance();
  if(ps->get<bool>("enable_pcl_debug_verbosity"))
    pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);

  initial_transform_WDF->setAlpha (ps->get<double>("alpha"));
  initial_transform_WDF->setCorrespondecesDFP (source_indices, target_indices);

  pcl::IterativeClosestPoint<PointNormal, PointNormal> icp_wdf;
  icp_wdf.setMaxCorrespondenceDistance (ps->get<double>("max_correspondence_dist"));
  icp_wdf.setMaximumIterations (ps->get<int>("max_iterations"));
  icp_wdf.setTransformationEpsilon (ps->get<double>("transformation_epsilon"));
  icp_wdf.setEuclideanFitnessEpsilon (ps->get<double>("euclidean_fitness_epsilon"));  //1
  // Set TransformationEstimationWDF as ICP transform estimator
  icp_wdf.setTransformationEstimation (initial_transform_WDF);

  checkforNaNs (source_cloud_normals_ptr);
  checkforNaNs (target_cloud_normals_ptr);
  icp_wdf.setInputCloud (source_cloud_normals_ptr);
  icp_wdf.setInputTarget (target_cloud_normals_ptr);

  PointCloudNormalsPtr cloud_transformed (new PointCloudNormals);
  if (ParameterServer::instance ()->get<bool> ("use_ransac_to_initialize_icp"))
    icp_wdf.align (*cloud_transformed, initial_transformation);
  else
    icp_wdf.align (*cloud_transformed);

  ROS_INFO_STREAM(
      "[SIIMCloudMatch::runICPMatch] Has converged? = " << icp_wdf.hasConverged ()
      << std::endl << " fitness score (SSD): " << icp_wdf.getFitnessScore (1000) <<
      " \n Final Transformation: \n" << icp_wdf.getFinalTransformation());

  return icp_wdf.getFinalTransformation ();
}
