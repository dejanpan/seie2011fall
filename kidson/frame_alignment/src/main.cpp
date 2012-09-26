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

  //-- Show detected matches
  cv::imshow ("Matches", img_matches);
  cv::moveWindow("Matches", -10, 0);

  RansacTransformation ransac_transformer;
  Eigen::Matrix4f ransac_trafo;
  float rmse = 0.0;
  ransac_transformer.getRelativeTransformationTo (source_feature_locations_3d,
      target_feature_locations_3d, &matches, ransac_trafo, rmse, good_matches,
      ParameterServer::instance ()->get<int> ("minimum_inliers"));

  ROS_INFO_STREAM("Final transformation from RANSAC: " << ransac_trafo);
  transformAndWriteToFile (source_cloud, ransac_trafo);

  cv::drawMatches (RGB_feature_detector.restoreCVMatFromPointCloud (source_cloud),
      source_keypoints, RGB_feature_detector.restoreCVMatFromPointCloud (target_cloud),
      target_keypoints, good_matches, img_matches, cv::Scalar::all (-1), cv::Scalar::all (-1),
      std::vector<char> (), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  //-- Show detected matches
  cv::imshow ("Good Matches", img_matches);
  cv::moveWindow("Good Matches", -10, 500);
  cv::waitKey(0);

  return 0;
}
