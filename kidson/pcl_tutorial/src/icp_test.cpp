#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_features.h>

int
 main (int argc, char** argv)
{
  if(argc != 3) return 0;
  //PointXYZRGBNormal
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_converg (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr featureCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indicies;

  //Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read (argv[1], *cloud_source);
  reader.read (argv[2], *cloud_target);
  std::cout << "PointCloud source has: " << cloud_source->points.size () << " data points." << std::endl; //*
  std::cout << "PointCloud target has: " << cloud_target->points.size () << " data points." << std::endl; //*

  pcl::IterativeClosestPointFeatures<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputCloud(cloud_source);
  icp.setInputTarget(cloud_target);
  icp.setSourceFeatures (featureCloud, indicies);
  icp.setTargetFeatures (featureCloud, indicies);
  icp.setFeatureErrorWeight(1);

  pcl::PointCloud<pcl::PointXYZRGB> Final;
  Eigen::Matrix4f guess;
  guess <<   1, 0, 0, 0.1,
		     0, 1, 0, 0,
		     0, 0, 1, 0.015,
		     0, 0, 0, 1;
  icp.align(Final, guess);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  transformPointCloud (*cloud_source, *cloud_converg, icp.getFinalTransformation());
  pcl::PCDWriter writer;
  writer.write ("converged.pcd", *cloud_converg, false);

 return (0);
}
