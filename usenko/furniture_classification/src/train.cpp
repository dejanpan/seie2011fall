/*
 * scan.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: vsu
 */

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/region_growing.h>

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile("data/scans/Aluminium_Group_EA_119_0000F152-centered/rotation0_distance4_tilt-15_shift0.pcd",
                       cloud);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setComputeNormals(true);

  // Set parameters
  mls.setInputCloud(cloud.makeShared());
  mls.setPolynomialFit(true);
  mls.setPolynomialOrder(2);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);

  // Reconstruct
  mls.process(mls_points);

  pcl::PointCloud<pcl::PointXYZ> mls_cloud;
  pcl::PointCloud<pcl::Normal> mls_normal;
  pcl::copyPointCloud(mls_points, mls_cloud);
  pcl::copyPointCloud(mls_points, mls_normal);

  std::vector<std::vector<int> > segment_indices;

  pcl::RegionGrowing<pcl::PointXYZ> region_growing;
  region_growing.setCloud(mls_cloud.makeShared());
  region_growing.setNormals(mls_normal.makeShared());
  region_growing.setNeighbourSearchMethod(tree);
  region_growing.setResidualTest(true);
  region_growing.setResidualThreshold(0.05);
  region_growing.setCurvatureTest(false);
  region_growing.setSmoothMode(false);
  region_growing.setSmoothnessThreshold(40*M_PI/180);
  region_growing.segmentPoints();
  segment_indices = region_growing.getSegments();

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segments;

  for (size_t i = 0; i < segment_indices.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr segment_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (size_t j = 0; j < segment_indices[i].size(); j++)
    {
      segment_cloud->points.push_back(cloud.points[segment_indices[i][j]]);
    }

    segment_cloud->is_dense = true;
    segment_cloud->width = segment_cloud->points.size();
    segment_cloud->height = 1;
    segments.push_back(segment_cloud);
  }

  pcl::visualization::PCLVisualizer viz;
  viz.initCameraParameters();
  viz.addCoordinateSystem(0.1);
  //viz.addPointCloudNormals<pcl::PointNormal>(mls_points.makeShared(), 10, 0.05);
  //viz.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (mls_cloud.makeShared(), mls_normal.makeShared(), 10, 0.05);


  for (size_t i = 0; i < segments.size(); i++)
  {
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_handler(segments[i]);
    std::strstream ss;
    ss << "cloud" << i;
    viz.addPointCloud(segments[i], color_handler, ss.str());
  }

  viz.spin();

  return 0;
}
