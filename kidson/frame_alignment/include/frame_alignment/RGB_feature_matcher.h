/*
 * RGB_feature_matcher.h
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#ifndef RGB_FEATURE_MATCHER_H_
#define RGB_FEATURE_MATCHER_H_

#include "frame_alignment/typedefs.h"
#include <cv.h>

class RGBFeatureMatcher
{
  public:
    RGBFeatureMatcher ();

    RGBFeatureMatcher (PointCloudPtr source_cloud_ptr, PointCloudPtr target_cloud_ptr);

    virtual ~RGBFeatureMatcher ();

    void setSourceCloud (const PointCloudPtr source_cloud);

    PointCloudConstPtr getSourceCloud ();

    void setTargetCloud (const PointCloudPtr target_cloud);

    PointCloudConstPtr getTargetCloud ();

    std::vector<Eigen::Vector4f> getSourceFeature3DLocations ();

    std::vector<Eigen::Vector4f> getTargetFeature3DLocations ();

    void findMatches (const cv::Mat& source_descriptors, const cv::Mat& target_descriptors,
        std::vector<cv::DMatch>& matches);

    void OutlierRemoval (const std::vector<cv::DMatch>& matches,
        std::vector<cv::DMatch>& good_matches);

    bool getMatches (std::vector<int>& source_indices_, std::vector<int>& target_indices_,
        Eigen::Matrix4f& trafo);

    void getIndicesFromMatches ();

  private:
    PointCloudConstPtr source_cloud_ptr_, target_cloud_ptr_;
    std::vector<Eigen::Vector4f> source_feature_3d_locations_, target_feature_3d_locations_;
};

#endif /* RGB_FEATURE_MATCHER_H_ */
