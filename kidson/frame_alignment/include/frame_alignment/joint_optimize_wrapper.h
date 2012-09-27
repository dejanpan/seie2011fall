/*
 * joint_optimize_wrapper.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#ifndef JOINT_OPTIMIZE_WRAPPER_CPP_
#define JOINT_OPTIMIZE_WRAPPER_CPP_

#include "frame_alignment/typedefs.h"

Eigen::Matrix4f performJointOptimization (PointCloudConstPtr source_cloud_ptr,
    PointCloudConstPtr target_cloud_ptr, std::vector<int>& source_indices,
    std::vector<int>& target_indices, Eigen::Matrix4f& initial_transformation);

#endif /* JOINT_OPTIMIZE_WRAPPER_CPP_ */
