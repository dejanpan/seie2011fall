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
#include "frame_alignment/ransac_transformation.h"
#include "frame_alignment/RGB_feature_detection.h"
#include "frame_alignment/typedefs.h"
#include "frame_alignment/transformation_estimation_wdf.h"
#include "frame_alignment/pcl_utils.h"

//opencv
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "frame_alignment");

  PointCloudPtr source_cloud (new PointCloud);
  PointCloudPtr target_cloud (new PointCloud);

  if (argc < 3)
  {
    ROS_WARN("Please provide 2 .pcd files: rosrun frame_matcher joint_optimization source.pcd target.pcd");
    exit (0);
  }
  pcl::PCDReader reader;
  reader.read (argv[1], *source_cloud);
  reader.read (argv[2], *target_cloud);

  // Extract RGB features and project into 3d
  RGBFeatureDetection RGB_feature_detector;
  std::vector<Eigen::Vector4f> source_feature_locations_3d, target_feature_locations_3d;
  std::vector<cv::KeyPoint> source_keypoints, target_keypoints;
  cv::Mat source_descriptors_2d, target_descriptors_2d;

  RGB_feature_detector.extractVisualFeaturesFromPointCloud (source_cloud, source_keypoints,
      source_descriptors_2d, source_feature_locations_3d);
  RGB_feature_detector.extractVisualFeaturesFromPointCloud (target_cloud, target_keypoints,
      target_descriptors_2d, target_feature_locations_3d);

  // Match features using opencv (doesn't consider depth info)
  std::vector<cv::DMatch> matches, good_matches;
  RGB_feature_detector.findMatches (source_descriptors_2d, target_descriptors_2d, matches);

  //-- Draw matches
  cv::Mat img_matches;
  cv::drawMatches (RGB_feature_detector.restoreCVMatFromPointCloud (source_cloud),
      source_keypoints, RGB_feature_detector.restoreCVMatFromPointCloud (target_cloud),
      target_keypoints, matches, img_matches, cv::Scalar::all (-1), cv::Scalar::all (-1),
      std::vector<char> (), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  cv::imshow ("Matches", img_matches);
  cv::moveWindow ("Matches", -10, 0);

  RansacTransformation ransac_transformer;
  Eigen::Matrix4f ransac_trafo;
  float rmse = 0.0;
  ransac_transformer.getRelativeTransformationTo (source_feature_locations_3d,
      target_feature_locations_3d, &matches, ransac_trafo, rmse, good_matches,
      ParameterServer::instance ()->get<int> ("minimum_inliers"));

  cv::drawMatches (RGB_feature_detector.restoreCVMatFromPointCloud (source_cloud),
      source_keypoints, RGB_feature_detector.restoreCVMatFromPointCloud (target_cloud),
      target_keypoints, good_matches, img_matches, cv::Scalar::all (-1), cv::Scalar::all (-1),
      std::vector<char> (), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  cv::imshow ("Good Matches", img_matches);
  cv::moveWindow ("Good Matches", -10, 500);
  cv::waitKey (0);

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
