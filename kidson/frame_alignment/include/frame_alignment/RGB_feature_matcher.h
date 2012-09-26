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

    void findMatches (const cv::Mat& source_descriptors, const cv::Mat& target_descriptors,
        std::vector<cv::DMatch>& matches);

    void OutlierRemoval (const std::vector<cv::DMatch>& matches,
        std::vector<cv::DMatch>& good_matches);

    bool getMatches (std::vector<int>& source_indices_, std::vector<int>& target_indices_,
        Eigen::Matrix4f& ransac_trafo);

    void getIndicesFromMatches (PointCloudConstPtr cloud_ptr, const std::vector<
        Eigen::Vector4f>& point_locations, std::vector<int>& indices);

  private:
    PointCloudConstPtr source_cloud_ptr_, target_cloud_ptr_;
};

#endif /* RGB_FEATURE_MATCHER_H_ */
