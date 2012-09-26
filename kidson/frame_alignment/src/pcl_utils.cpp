/*
 * pcl_utils.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#include "frame_alignment/typedefs.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

void transformAndWriteToFile (const PointCloudConstPtr cloud_in, const Eigen::Matrix4f& trafo)
{
  pcl::PCDReader reader;
  PointCloudPtr cloud_out (new PointCloud);

  pcl::transformPointCloud (*cloud_in, *cloud_out, trafo);

  pcl::PCDWriter writer;
  writer.write ("transformed_cloud.pcd", *cloud_out, false);
}
