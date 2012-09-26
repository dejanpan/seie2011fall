/*
 * joint_optimize_wrapper.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#include "frame_alignment/joint_optimize_wrapper.h"
#include "frame_alignment/transformation_estimation_wdf.h"

Eigen::Matrix4f performJointOptimization (PointCloudConstPtr source_cloud_ptr,
    PointCloudConstPtr target_cloud_ptr, std::vector<int>& source_indices,
    std::vector<int>& target_indices, Eigen::Matrix4f& inital_transformation)
{
//  boost::shared_ptr<TransformationEstimationWDF<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> >
//      initialTransformWDF (new TransformationEstimationWDF<pcl::PointXYZRGBNormal,
//          pcl::PointXYZRGBNormal> ());
//
//  initialTransformWDF->setAlpha (0.5);
//  initialTransformWDF->setCorrespondecesDFP (source_indices, target_indices);
//
//  // Instantiate ICP
  pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp_wdf;
//
//  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
//  icp_wdf.setMaxCorrespondenceDistance (0.05);
//  // Set the maximum number of iterations (criterion 1)
//  icp_wdf.setMaximumIterations (75);
//  // Set the transformation epsilon (criterion 2)
//  icp_wdf.setTransformationEpsilon (1e-8);
//  // Set the euclidean distance difference epsilon (criterion 3)
//  icp_wdf.setEuclideanFitnessEpsilon (0); //1
//
//  // Set TransformationEstimationWDF as ICP transform estimator
//  icp_wdf.setTransformationEstimation (initialTransformWDF);
//
//  icp_wdf.setInputCloud (concatinatedSourceCloud);
//  icp_wdf.setInputTarget (concatinatedTargetCloud);
//
//  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_transformed (new pcl::PointCloud<
//      pcl::PointXYZRGBNormal>);
//  // As before, due to my initial bad naming, it is the "target" that is being transformed
//  //                  set initial transform
//  ROS_INFO_STREAM ("---------------------------------------------------------      indices size: "
//      << indicesSource.size ());
//  if (indicesSource.size () < MINIMUM_FEATURES)
//    icp_wdf.align (*cloud_transformed);
//  else
//    icp_wdf.align (*cloud_transformed, ransacInverse);
//  std::cout << "[SIIMCloudMatch::runICPMatch] Has converged? = " << icp_wdf.hasConverged ()
//      << std::endl << " fitness score (SSD): " << icp_wdf.getFitnessScore (1000) << std::endl;
//  icp_wdf.getFinalTransformation ();

  return icp_wdf.getFinalTransformation ();
}
