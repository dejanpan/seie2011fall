/*
 * RGBFeatureDetection.h
 *
 *  Created on: Sep 25, 2012
 *      Author: kidson
 */

#ifndef RGBFEATUREDETECTION_H_
#define RGBFEATUREDETECTION_H_

#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include "frame_alignment/typedefs.h"

class RGBFeatureDetection
{
  public:
    RGBFeatureDetection ();
    virtual ~RGBFeatureDetection ();

    cv::Mat restoreCVMatFromPointCloud (PointCloudConstPtr cloud_in);

    void projectFeaturesTo3D (std::vector<cv::KeyPoint>& feature_locations_2d,
        std::vector<Eigen::Vector4f> & feature_locations_3d,
        PointCloudConstPtr point_cloud);

    void extractVisualFeaturesFromPointCloud (PointCloudConstPtr input_cloud, std::vector<
        cv::KeyPoint>& keypoints, cv::Mat& descriptors_2d,
        std::vector<Eigen::Vector4f>& features_3d);

    void extractVisualFeaturesFromPointCloud (PointCloudConstPtr input_cloud, std::vector<
        Eigen::Vector4f>& features_3d);

    void findMatches (const cv::Mat& source_descriptors,
        const cv::Mat& target_descriptors, std::vector<cv::DMatch>& good_matches);

    void OutlierRemoval (const std::vector<cv::DMatch>& matches,
         std::vector<cv::DMatch>& good_matches);

  private:
    int image_counter_;
};

#endif /* RGBFEATUREDETECTION_H_ */
