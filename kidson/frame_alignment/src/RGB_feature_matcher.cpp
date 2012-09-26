/*
 * RGB_feature_matcher.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#include "frame_alignment/RGB_feature_matcher.h"
#include "frame_alignment/RGB_feature_detection.h"
#include "frame_alignment/ransac_transformation.h"
#include "frame_alignment/parameter_server.h"

#include "opencv2/highgui/highgui.hpp"

RGBFeatureMatcher::RGBFeatureMatcher (PointCloudPtr source_cloud_ptr,
    PointCloudPtr target_cloud_ptr) :
  source_cloud_ptr_ (source_cloud_ptr), target_cloud_ptr_ (target_cloud_ptr)
{
}

RGBFeatureMatcher::RGBFeatureMatcher ()
{
  // TODO Auto-generated destructor stub
}

RGBFeatureMatcher::~RGBFeatureMatcher ()
{
  // TODO Auto-generated destructor stub
}

void RGBFeatureMatcher::setSourceCloud (const PointCloudPtr source_cloud)
{
  source_cloud_ptr_ = source_cloud;
}

PointCloudConstPtr RGBFeatureMatcher::getSourceCloud ()
{
  return source_cloud_ptr_;
}

void RGBFeatureMatcher::setTargetCloud (const PointCloudPtr target_cloud)
{
  target_cloud_ptr_ = target_cloud;
}

PointCloudConstPtr RGBFeatureMatcher::getTargetCloud ()
{
  return target_cloud_ptr_;
}

std::vector<Eigen::Vector4f> RGBFeatureMatcher::getSourceFeature3DLocations ()
{
  return source_feature_3d_locations_;
}

std::vector<Eigen::Vector4f> RGBFeatureMatcher::getTargetFeature3DLocations ()
{
  return target_feature_3d_locations_;
}

bool RGBFeatureMatcher::getMatches (std::vector<int>& source_indices_,
    std::vector<int>& target_indices_, Eigen::Matrix4f& trafo)
{
  // Extract RGB features and project into 3d
  RGBFeatureDetection RGB_feature_detector;
  std::vector<cv::KeyPoint> source_keypoints, target_keypoints;
  cv::Mat source_descriptors, target_descriptors;

  RGB_feature_detector.extractVisualFeaturesFromPointCloud (source_cloud_ptr_, source_keypoints,
      source_descriptors, source_feature_3d_locations_);
  RGB_feature_detector.extractVisualFeaturesFromPointCloud (target_cloud_ptr_, target_keypoints,
      target_descriptors, target_feature_3d_locations_);

  // Match features using opencv (doesn't consider depth info)
  std::vector<cv::DMatch> matches, good_matches;
  this->findMatches (source_descriptors, target_descriptors, matches);

  RansacTransformation ransac_transformer;
  Eigen::Matrix4f ransac_trafo;
  float rmse = 0.0;
  ransac_transformer.getRelativeTransformationTo (source_feature_3d_locations_,
      target_feature_3d_locations_, &matches, ransac_trafo, rmse, good_matches,
      ParameterServer::instance ()->get<int> ("minimum_inliers"));

  if (ParameterServer::instance ()->get<bool> ("show_feature_matching"))
  {
    cv::Mat img_matches;
    cv::drawMatches (RGB_feature_detector.restoreCVMatFromPointCloud (source_cloud_ptr_),
        source_keypoints, RGB_feature_detector.restoreCVMatFromPointCloud (target_cloud_ptr_),
        target_keypoints, matches, img_matches, cv::Scalar::all (-1), cv::Scalar::all (-1),
        std::vector<char> (), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imshow ("Matches", img_matches);
    cv::moveWindow ("Matches", -10, 0);

    cv::drawMatches (RGB_feature_detector.restoreCVMatFromPointCloud (source_cloud_ptr_),
        source_keypoints, RGB_feature_detector.restoreCVMatFromPointCloud (target_cloud_ptr_),
        target_keypoints, good_matches, img_matches, cv::Scalar::all (-1), cv::Scalar::all (-1),
        std::vector<char> (), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imshow ("Good Matches", img_matches);
    cv::moveWindow ("Good Matches", -10, 500);
    cv::waitKey (0);
    cv::destroyAllWindows ();
  }
}

void RGBFeatureMatcher::getIndicesFromMatches ()
{


}

void RGBFeatureMatcher::findMatches (const cv::Mat& source_descriptors,
    const cv::Mat& target_descriptors, std::vector<cv::DMatch>& matches)
{
  cv::DescriptorMatcher* matcher;
  if (ParameterServer::instance ()->get<std::string> ("descriptor_matcher") == "FLANN")
    matcher = new cv::FlannBasedMatcher;
  else if (ParameterServer::instance ()->get<std::string> ("descriptor_matcher") == "Bruteforce")
    matcher = new cv::BFMatcher (cv::NORM_L1, false);
  else
  {
    ROS_WARN ("descriptor_matcher parameter not correctly set, defaulting to FLANN");
    matcher = new cv::FlannBasedMatcher;
  }
  matcher->match (source_descriptors, target_descriptors, matches);
}

// crude outlier removal implementation.  RANSAC is preferred to find outliers

void RGBFeatureMatcher::OutlierRemoval (const std::vector<cv::DMatch>& matches, std::vector<
    cv::DMatch>& good_matches)
{
  // Outlier detection
  double max_dist = 0;
  double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for (uint i = 0; i < matches.size (); i++)
  {
    double dist = matches[i].distance;
    if (dist < min_dist)
      min_dist = dist;
    if (dist > max_dist)
      max_dist = dist;
  }

  printf ("-- Max dist : %f \n", max_dist);
  printf ("-- Min dist : %f \n", min_dist);

  //-- Find only "good" matches (i.e. whose distance is less than 2*min_dist )
  //-- PS.- radiusMatch can also be used here.
  for (uint i = 0; i < matches.size (); i++)
  {
    if (matches[i].distance < 4 * min_dist)
      good_matches.push_back (matches[i]);
  }
  for (uint i = 0; i < good_matches.size (); i++)
  {
    printf ("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i,
        good_matches[i].queryIdx, good_matches[i].trainIdx);
  }
}
