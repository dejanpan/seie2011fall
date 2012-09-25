/*
 * joint_optimization.cpp
 *
 *  Created on: 11.07.2012
 *      Author: ross
 */

#include <stdio.h>
#include <iostream>

//local files
#include "sift_gpu_wrapper.h"
#include "parameter_server.h"
#include "ransac_transformation.h"
#include "RGBFeatureDetection.h"
#include "typedefs.h"
#include "transformation_estimation_wdf.h"

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//opencv
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

int main (int argc, char** argv)
{
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

  RGBFeatureDetection RGB_feature_detector;
  // Extract RGB features and project into 3d
  std::vector<Eigen::Vector4f> source_features_3d, target_features_3d;
  cv::Mat source_features_2d, target_features_2d;
  std::vector<cv::KeyPoint> source_keypoints, target_keypoints;

  RGB_feature_detector.extractVisualFeaturesFromPointCloud (source_cloud, source_keypoints,
      source_features_2d, source_features_3d);
  RGB_feature_detector.extractVisualFeaturesFromPointCloud (target_cloud, target_keypoints,
      target_features_2d, target_features_3d);

  // Match features using opencv (doesn't consider depth info)
  std::vector<cv::DMatch> good_matches;
  RGB_feature_detector.flannMatcher (source_features_2d, target_features_2d, good_matches);

  //-- Draw only "good" matches
  cv::Mat img_matches;
  cv::drawMatches (RGB_feature_detector.restoreCVMatFromPointCloud (source_cloud),
      source_keypoints, RGB_feature_detector.restoreCVMatFromPointCloud (target_cloud),
      target_keypoints, good_matches, img_matches, cv::Scalar::all (-1), cv::Scalar::all (-1),
      std::vector<char> (), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  //-- Show detected matches
  cv::imshow ("Good Matches", img_matches);
  cv::waitKey (0);
  return 0;
}
