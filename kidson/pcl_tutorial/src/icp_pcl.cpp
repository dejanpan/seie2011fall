#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
 
  // Fill in the CloudIn data
  cloud_in->width    = 4;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);



	cloud_in->points[0].x = 7;
	cloud_in->points[0].y = 0;
	cloud_in->points[1].x = 3;
	cloud_in->points[1].y = 2;
	cloud_in->points[2].x = 9;
	cloud_in->points[2].y = 5;
	cloud_in->points[3].x = 13;
	cloud_in->points[3].y = 11;
	cloud_in->points[0].z = 0;
	cloud_in->points[2].z = 0;
	cloud_in->points[1].z = 0;
	cloud_in->points[3].z = 0;

	*cloud_out = *cloud_in;

//  cloud_out->width    = 4;
//  cloud_out->height   = 1;
//  cloud_out->is_dense = false;
//  cloud_out->points.resize (cloud_out->width * cloud_out->height);

//	cloud_out->points[0].x = 2;
//	cloud_out->points[0].y = 2;
//	cloud_out->points[1].x = 6;
//	cloud_out->points[1].y = 2;
//	cloud_out->points[2].x = 6;
//	cloud_out->points[2].y = 6;
//	cloud_out->points[3].x = 6;
//	cloud_out->points[3].y = 8;
//	cloud_out->points[0].z = 0;
//	cloud_out->points[1].z = 0;
//	cloud_out->points[2].z = 0;
//	cloud_out->points[3].z = 0;

  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"      << std::endl;

  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
	  std::cout << "    " <<
      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " << cloud_in->points[i].z << std::endl;
  }

  std::cout << "size:" << cloud_out->points.size() << std::endl;

  for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

  //Fill in the cloud data
  pcl::PCDReader reader;
//  reader.read (argv[1], *cloud_in);
//  reader.read (argv[2], *cloud_out);
  std::cout << "PointCloud source has: " << cloud_in->points.size () << " data points." << std::endl;
  std::cout << "PointCloud target has: " << cloud_out->points.size () << " data points." << std::endl;

//  Eigen::Matrix4f guess;
//  guess << 1,   0,  0,  0.0,
//		   0,	1,	0,	0.0,
//		   0,	0,	1,	0.0,
//		   0,	0,	0,	1;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setRANSACOutlierRejectionThreshold(1000000.0);
  icp.setMaximumIterations (1);
  icp.setMaxCorrespondenceDistance(4.0);
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  pcl::PCDWriter writer;
    writer.write ("converged.pcd", Final, false);
    //writer.write ("cloud_out.pcd", *cloud_out, false);

 return (0);
}
