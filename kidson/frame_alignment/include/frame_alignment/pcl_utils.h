/*
 * pcl_utils.h
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#ifndef PCL_UTILS_H_
#define PCL_UTILS_H_


void transformAndWriteToFile (const PointCloudConstPtr cloud_in, const Eigen::Matrix4f& trafo);

#endif /* PCL_UTILS_H_ */
