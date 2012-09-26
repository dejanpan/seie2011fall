/*
 * main.cpp
 *
 *  Created on: 11.07.2012
 *      Author: ross Kidson
 */

#include <stdio.h>
#include <iostream>

//local files
//#include "frame_alignment/sift_gpu_wrapper.h"
#include "frame_alignment/parameter_server.h"
#include "frame_alignment/RGB_feature_matcher.h"
#include "frame_alignment/typedefs.h"
#include "frame_alignment/transformation_estimation_wdf.h"
#include "frame_alignment/pcl_utils.h"

//opencv
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

int main (int argc, char** argv)
{
  if (argc < 3)
  {
    ROS_WARN("Please provide 2 .pcd files: rosrun frame_matcher joint_optimization source.pcd target.pcd");
    exit (0);
  }
  ros::init (argc, argv, "frame_alignment");
  PointCloudPtr source_cloud_ptr (new PointCloud);
  PointCloudPtr target_cloud_ptr (new PointCloud);
  pcl::PCDReader reader;
  reader.read (argv[1], *source_cloud_ptr);
  reader.read (argv[2], *target_cloud_ptr);

  RGBFeatureMatcher point_cloud_matcher (source_cloud_ptr, target_cloud_ptr);
  std::vector<int> source_feature_indices, target_feature_indices;
  Eigen::Matrix4f ransac_trafo;
  point_cloud_matcher.getMatches (source_feature_indices, target_feature_indices, ransac_trafo);

  // PERFORM RGBD-ICP JOINT OPTIMIZATION --------------------------

  //  boost::shared_ptr<TransformationEstimationWDF<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> >
  //      initialTransformWDF (new TransformationEstimationWDF<pcl::PointXYZRGBNormal,
  //          pcl::PointXYZRGBNormal> ());
  //
  //  initialTransformWDF->setAlpha (alpha);
  //  initialTransformWDF->setCorrespondecesDFP (indicesSource, indicesTarget);
  //
  //  // Instantiate ICP
  //  pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp_wdf;
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
  //  ROS_INFO_STREAM("---------------------------------------------------------      indices size: " << indicesSource.size() );
  //  if (indicesSource.size () < MINIMUM_FEATURES)
  //    icp_wdf.align (*cloud_transformed);
  //  else
  //    icp_wdf.align (*cloud_transformed, ransacInverse);
  //  std::cout << "[SIIMCloudMatch::runICPMatch] Has converged? = " << icp_wdf.hasConverged ()
  //      << std::endl << " fitness score (SSD): " << icp_wdf.getFitnessScore (1000) << std::endl;
  //  icp_wdf.getFinalTransformation ();

  return 0;
}
