/*
 * RGBFeatureDetection.cpp
 *
 *  Created on: Sep 25, 2012
 *      Author: kidson
 */

#include "RGBFeatureDetection.h"

//opencv
#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cv.h>

#include <ros/console.h>

RGBFeatureDetection::RGBFeatureDetection () :
  image_counter_ (0)
{
}

RGBFeatureDetection::~RGBFeatureDetection ()
{
}

cv::Mat RGBFeatureDetection::restoreCVMatFromPointCloud (PointCloudConstPtr cloud_in)
{
  cv::Mat restored_image = cv::Mat (cloud_in->height, cloud_in->width, CV_8UC3);
  for (uint rows = 0; rows < cloud_in->height; rows++)
  {
    for (uint cols = 0; cols < cloud_in->width; ++cols)
    {
      //      restored_image.at<uint8_t>(rows, cols) = cloud_in->at(cols, rows).r;
      restored_image.at<cv::Vec3b> (rows, cols)[0] = cloud_in->at (cols, rows).b;
      restored_image.at<cv::Vec3b> (rows, cols)[1] = cloud_in->at (cols, rows).g;
      restored_image.at<cv::Vec3b> (rows, cols)[2] = cloud_in->at (cols, rows).r;
    }
  }
  return restored_image;
}

// projectFeaturesTo3D
//
// Takes a RGB feature pixel location and uses depth information to make it a 3d coordiant
// this also removes features that have nan as a depth value

void RGBFeatureDetection::projectFeaturesTo3D (std::vector<cv::KeyPoint>& feature_locations_2d,
    std::vector<Eigen::Vector4f> & feature_locations_3d, PointCloudConstPtr point_cloud)
{
  int index = -1;
  for (unsigned int i = 0; i < feature_locations_2d.size (); /*increment at end of loop*/)
  {
    ++index;

    cv::Point2f p2d = feature_locations_2d[i].pt;
    PointType p3d = point_cloud->at ((int) p2d.x, (int) p2d.y);

    // Check for invalid measurements
    if (isnan (p3d.x) || isnan (p3d.y) || isnan (p3d.z))
    {
      ROS_DEBUG ("Feature %d has been extracted at NaN depth. Omitting", i);
      feature_locations_2d.erase (feature_locations_2d.begin () + i);
      continue;
    }

    feature_locations_3d.push_back (Eigen::Vector4f (p3d.x, p3d.y, p3d.z, 1.0));
    //featuresUsed.push_back(index);  //save id for constructing the descriptor matrix
    i++; //Only increment if no element is removed from vector
  }
}

void RGBFeatureDetection::extractVisualFeaturesFromPointCloud (PointCloudPtr input_cloud,
    std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors_2d,
    std::vector<Eigen::Vector4f>& features_3d)
{
  // get image from pointcloud
  cv::Mat input_image = restoreCVMatFromPointCloud (input_cloud);

  // convert to black and white
  cv::Mat image_greyscale;
  cvtColor (input_image, image_greyscale, CV_RGB2GRAY);

  //detect sift features
  cv::SiftFeatureDetector detector;
  detector.detect (image_greyscale, keypoints);

  // draw features (debugging)
  cv::Mat output;
  cv::drawKeypoints (image_greyscale, keypoints, output);
  std::stringstream result;
  result << "sift_result" << image_counter_++ << ".jpg";
  cv::imwrite (result.str(), output);

  // get sift descriptors
  cv::SiftDescriptorExtractor extractor;
  extractor.compute (image_greyscale, keypoints, descriptors_2d);

  // project sift descriptors to 3d
  projectFeaturesTo3D (keypoints, features_3d, input_cloud);
}

void RGBFeatureDetection::extractVisualFeaturesFromPointCloud (PointCloudPtr input_cloud,
    std::vector<Eigen::Vector4f>& features_3d)
{
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  extractVisualFeaturesFromPointCloud (input_cloud, keypoints, descriptors, features_3d);
}

void RGBFeatureDetection::flannMatcher (const cv::Mat& source_descriptors,
    const cv::Mat& target_descriptors, std::vector<cv::DMatch>& good_matches)
{
  // Matching descriptor vectors using FLANN matcher
  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> matches;
  matcher.match (source_descriptors, target_descriptors, matches);

  // Outlier detection
  double max_dist = 0;
  double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for (int i = 0; i < source_descriptors.rows; i++)
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
  for (int i = 0; i < source_descriptors.rows; i++)
  {
    if (matches[i].distance < 3 * min_dist)
      good_matches.push_back (matches[i]);
  }
  for (uint i = 0; i < good_matches.size (); i++)
  {
    printf ("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i,
        good_matches[i].queryIdx, good_matches[i].trainIdx);
  }

}
