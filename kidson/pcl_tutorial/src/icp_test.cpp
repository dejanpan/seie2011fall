#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
 main (int argc, char** argv)
{
  if(argc != 3) return 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_converg (new pcl::PointCloud<pcl::PointXYZ>);

  //Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read (argv[1], *cloud_source);
  reader.read (argv[2], *cloud_target);
  std::cout << "PointCloud source has: " << cloud_source->points.size () << " data points." << std::endl; //*
  std::cout << "PointCloud target has: " << cloud_target->points.size () << " data points." << std::endl; //*

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_source);
  icp.setInputTarget(cloud_target);
  pcl::PointCloud<pcl::PointXYZ> Final;
  Eigen::Matrix4f guess;
  guess <<   1, 0, 0, 0.1,
		     0, 1, 0, 0,
		     0, 0, 1, 0.015,
		     0, 0, 0, 1;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  transformPointCloud (*cloud_source, *cloud_converg, icp.getFinalTransformation());
  pcl::PCDWriter writer;
  writer.write ("converged.pcd", *cloud_converg, false);

 return (0);
}
