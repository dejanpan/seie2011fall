#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

int
main (int argc, char** argv)
{
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Matrix4f trafo;
	reader.read (argv[1], *cloudIn);
    std::ifstream myfile;
    myfile.open(argv[2]);
	myfile >> trafo(0,0);
	myfile >> trafo(0,1);
	myfile >> trafo(0,2);
	myfile >> trafo(0,3);
	myfile >> trafo(1,0);
	myfile >> trafo(1,1);
	myfile >> trafo(1,2);
	myfile >> trafo(1,3);
	myfile >> trafo(2,0);
	myfile >> trafo(2,1);
	myfile >> trafo(2,2);
	myfile >> trafo(2,3);
	myfile >> trafo(3,0);
	myfile >> trafo(3,1);
	myfile >> trafo(3,2);
	myfile >> trafo(3,3);

	transformPointCloud (*cloudIn, *cloudOut,  trafo);

	pcl::PCDWriter writer;
	writer.write ("output.pcd", *cloudOut, false);
  return (0);
}
