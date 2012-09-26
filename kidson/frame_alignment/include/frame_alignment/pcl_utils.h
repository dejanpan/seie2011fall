/*
 * pcl_utils.h
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#ifndef PCL_UTILS_H_
#define PCL_UTILS_H_


void transformAndWriteToFile (const PointCloudConstPtr cloud_in, const Eigen::Matrix4f& trafo);

void writeFeaturePointCloudsToFile (const PointCloudConstPtr source_cloud,
    const std::vector<int>& source_indices, const PointCloudConstPtr target_cloud,
    const std::vector<int> target_indices, const Eigen::Matrix4f& trafo);

#endif /* PCL_UTILS_H_ */
