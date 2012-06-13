#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>

int
main (int argc, char** argv)
{
  if(argc != 2)
  {
	  std::cout << "Not enough arguments, please provide a point clouds \n";
	  return 0;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PCDReader reader;
  reader.read (argv[1], *cloud);

  // Create the normal estimation class, and pass the input dataset to it
  //pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
  ne.setNumberOfThreads(4);
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  //pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  //ne.setSearchMethod (tree);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr normals_tree_ (new pcl::search::KdTree<pcl::PointXYZRGB>);
  ne.setKSearch(30);
  ne.setSearchMethod(normals_tree_);

  // Output datasets
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  // Use all neighbors in a sphere of radius 3cm
  //ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  pcl::copyPointCloud (*cloud, *cloud_normals);

  pcl::PCDWriter writer;
  writer.write ("normals.pcd", *cloud_normals, false);

  //std::cout << cloud_normals->points[5].getNormalVector4fMap() ;

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}
