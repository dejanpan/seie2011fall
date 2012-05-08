#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include "pcl/segmentation/extract_clusters.h"

typedef pcl::PointXYZRGB Point;
typedef pcl::search::KdTree<Point> KdTree;
typedef KdTree::Ptr KdTreePtr;

int
 main (int argc, char** argv)
{
  pcl::PointCloud<Point>::Ptr cloud_in (new pcl::PointCloud<Point>);

  //Read in cloud
  pcl::PCDReader reader;
  reader.read (argv[1], *cloud_in);
  std::cout << "PointCloud source has: " << cloud_in->points.size () << " data points." << std::endl;

  // run plane extraction to obtain indices of my pointcloud
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::SACSegmentation<Point> seg;
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.02);
  seg.setInputCloud (cloud_in);
  seg.segment (*inliers, *coefficients);

  pcl::PCDWriter writer;
  //The following is to test the segmentation works, and the indices are good, which they are
  writer.write ("extractedPlane.pcd", *cloud_in, inliers->indices, true);

  //now cluster using indices as the input
  pcl::EuclideanClusterExtraction<Point> cluster_;
  std::vector<pcl::PointIndices> clusters_;
  std::cout << "test1\n" ;
  KdTreePtr clusters_tree_(new KdTree);
  std::cout << "test2\n" ;
  clusters_tree_->setEpsilon(1);
  std::cout << "test3\n" ;
  //cluster_.setClusterTolerance(0.03);
  cluster_.setMinClusterSize(200);
  std::cout << "test4\n" ;
  //cluster_.setSearchMethod(clusters_tree_);
  std::cout << "test5\n" ;
  cluster_.setInputCloud(cloud_in);
  cluster_.setIndices(inliers);
  std::cout << "test6\n" ;
  cluster_.extract(clusters_);
  std::cout << "test7\n" ;
  ROS_INFO("Found %d clusters.", (int)clusters_.size ());

 return (0);
}
