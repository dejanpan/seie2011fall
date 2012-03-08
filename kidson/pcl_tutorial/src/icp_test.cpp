#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_features.h>
#include <pcl/features/normal_3d.h>

//rosbag stuff:
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "pcl_tutorial/featureMatch.h"
#include "pcl_tutorial/match.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;

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


void getTestDataFromBag(PointCloudPtr cloud_source, PointCloudPtr cloud_target,
		PointCloudNormalPtr featureCloudSource, std::vector<int> &indicesSource,
		PointCloudNormalPtr featureCloudTarget, std::vector<int> &indicesTarget,
		Eigen::Matrix4f &initialTransform, int rosMessageNumber)
{
	rosbag::Bag bag;
	bag.open("test.bag", rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(std::string("/feature_match_out_topic"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		pcl_tutorial::featureMatch::ConstPtr fm = m.instantiate<pcl_tutorial::featureMatch>();
		ROS_INFO("test: %i", fm->matches[4].queryId);

	}

	bag.close();
}

int
 main (int argc, char** argv)
{
  if(argc != 3)
  {
	  std::cout << "Not enough arguments, please provide two point clouds \n";
	  return 0;
  }


  PointCloudPtr cloud_source (new PointCloud);
  PointCloudPtr cloud_target (new PointCloud);
  PointCloudPtr cloud_converg (new PointCloud);
  PointCloudNormalPtr cloud_source_normals (new PointCloudNormal);
  PointCloudNormalPtr cloud_target_normals (new PointCloudNormal);
  PointCloudNormalPtr featureCloudSource (new PointCloudNormal);
  PointCloudNormalPtr featureCloudTarget (new PointCloudNormal);
  Eigen::Matrix4f initialTransform;
  std::vector<int> indicesSource;
  std::vector<int> indicesTarget;

  //Fill in the cloud data
  /*pcl::PCDReader reader;
  reader.read (argv[1], *cloud_source);
  reader.read (argv[2], *cloud_target);
  std::cout << "PointCloud source has: " << cloud_source->points.size () << " data points." << std::endl;
  std::cout << "PointCloud target has: " << cloud_target->points.size () << " data points." << std::endl;
  */

  getTestDataFromBag(cloud_source, cloud_target, featureCloudSource, indicesSource, featureCloudTarget, indicesTarget, initialTransform, 5);

  //calculate normals
  normalEstimation(cloud_source, cloud_source_normals);
  normalEstimation(cloud_target, cloud_target_normals);

  pcl::IterativeClosestPointFeatures<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
  icp.setInputCloud(cloud_source_normals);
  icp.setInputTarget(cloud_target_normals);
  icp.setSourceFeatures (featureCloudSource, indicesSource);
  icp.setTargetFeatures (featureCloudTarget, indicesTarget);
  icp.setFeatureErrorWeight(1);

  PointCloudNormal Final;
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


