#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


int
main (int argc, char** argv)
{
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSource (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
	reader.read (argv[1], *cloudSource);

	  Eigen::Matrix4f guess;
	  guess << 1,   0.2,  0.3,  -0.3,
			   -0.2,	1,	-0.2,	0.9,
			   -0.3,	0.2,	1,	0.4,
			   0,	0,	0,	1;
	//  guess <<	0.998083, -0.0236136,  -0.057204, 0.068627,
	//			0.017266,    0.99389,   -0.10902, 0.627856,
	//			0.0594288,   0.107824,   0.992392,   0.370651,
	//			0,          0,          0,          1;
	//  guess  << 0.996215, 0.00456522, -0.0868249,   0.122714,
	//-0.0134495,   0.994693, -0.102021,   0.614769,
	// 0.0858986,   0.102803,   0.990987,   0.349081,
	//         0,        0,       0,        1;

	transformPointCloud (*cloudSource, *cloud_out,  guess);

	pcl::PCDWriter writer;
	writer.write ("output.pcd", *cloud_out, false);
  return (0);
}
