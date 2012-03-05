#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
 main (int argc, char** argv)
{
  if(argc != 3) return 0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGB>);

  //Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read (argv[1], *cloud_source); // Remember to download the file first!
  reader.read (argv[2], *cloud_target); // Remember to download the file first!

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputCloud(cloud_source);
  icp.setInputTarget(cloud_target);
  pcl::PointCloud<pcl::PointXYZRGB> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;


 return (0);
}
