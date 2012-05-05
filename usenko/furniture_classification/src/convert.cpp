/*
 * scan.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: vsu
 */

#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <training.h>

int main(int argc, char** argv) {

	if (argc < 5) {
		PCL_INFO ("Usage %s -input_file /in_file -output_file /out_file [options]\n", argv[0]);
		PCL_INFO (" * where options are:\n"
				"         -tilt <X>  : tilt. Default : 30\n"
				"");
		return -1;
	}

	int tilt = 30;
	std::string input_file;
	std::string output_file;

	pcl::console::parse_argument(argc, argv, "-input_file", input_file);
	pcl::console::parse_argument(argc, argv, "-output_file", output_file);
	pcl::console::parse_argument(argc, argv, "-tilt", tilt);



	pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud_transformed;

	pcl::io::loadPCDFile(input_file, cloud);

	Eigen::Affine3f view_transform;
	view_transform.matrix() <<  0,  0,  1, 0,
							   -1,  0,  0, 0,
							    0, -1,  0, 0,
							    0,  0,  0, 1;

	Eigen::AngleAxis<float> rot(tilt*M_PI/180, Eigen::Vector3f(0,1,0));

	view_transform.prerotate(rot);

	pcl::transformPointCloud(cloud, cloud_transformed, view_transform);

	pcl::io::savePCDFile(output_file, cloud_transformed);

	return 0;
}

