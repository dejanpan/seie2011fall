/*
 * pcl_utils.h
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#ifndef PCL_UTILS_H_
#define PCL_UTILS_H_

#include "frame_alignment/typedefs.h"

void transformAndWriteToFile (const PointCloudConstPtr cloud_in, const Eigen::Matrix4f& trafo);

void writeFeaturePointCloudsToFile (const PointCloudConstPtr source_cloud,
    const std::vector<int>& source_indices, const PointCloudConstPtr target_cloud,
    const std::vector<int> target_indices, const Eigen::Matrix4f& trafo);

void writePCDToFile (const std::string& fileName, const PointCloudConstPtr cloud_ptr);

void writePCDToFile (const std::string& fileName, const PointCloudConstPtr cloud_ptr,
    const std::vector<int>& indices);

void calculatePointCloudNormals (const PointCloudConstPtr input_cloud_ptr,
    PointCloudNormalsPtr output_cloud_ptr);

#endif /* PCL_UTILS_H_ */
