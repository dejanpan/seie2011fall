#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_features.h>
#include <pcl/features/normal_3d.h>


void normalEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudIn, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloudOut)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud (pointCloudIn);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03);
  ne.compute (*pointCloudOut);
  pcl::copyPointCloud (*pointCloudIn, *pointCloudOut);
}

int
 main (int argc, char** argv)
{
  if(argc != 3)
  {
	  std::cout << "Not enough arguments, please provide two point clouds \n";
	  return 0;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_converg (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr featureCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  std::vector<int> indicies;

  //Fill in the cloud data
  pcl::PCDReader reader;
  reader.read (argv[1], *cloud_source);
  reader.read (argv[2], *cloud_target);
  std::cout << "PointCloud source has: " << cloud_source->points.size () << " data points." << std::endl; //*
  std::cout << "PointCloud target has: " << cloud_target->points.size () << " data points." << std::endl; //*

  //calculate normals
  normalEstimation(cloud_source, cloud_source_normals);
  normalEstimation(cloud_target, cloud_target_normals);

  pcl::IterativeClosestPointFeatures<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
  icp.setInputCloud(cloud_source_normals);
  icp.setInputTarget(cloud_target_normals);
  icp.setSourceFeatures (featureCloud, indicies);
  icp.setTargetFeatures (featureCloud, indicies);
  icp.setFeatureErrorWeight(1);

  pcl::PointCloud<pcl::PointXYZRGBNormal> Final;
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
