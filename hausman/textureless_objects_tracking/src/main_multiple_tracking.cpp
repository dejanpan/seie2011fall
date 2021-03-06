/*
 * main_multiple_tracking.cpp
 *
 *  Created on: Jun 11, 2012
 *      Author: Karol Hausman
 */

#include "3d_tracking_lines.cpp"
#include <ros/ros.h>



void usage(char** argv) {
	std::cout << "usage: " << argv[0] << " <device_id> [-C] [-g]\n\n";
	std::cout
			<< "  -C:  initialize the pointcloud to track without plane segmentation"
			<< std::endl;
	std::cout << "  -D: visualizing with non-downsampled pointclouds."
			<< std::endl;
	std::cout << "  -P: not visualizing particle cloud." << std::endl;
	std::cout << "  -fixed: use the fixed number of the particles."
			<< std::endl;
	std::cout
			<< "  -d <value>: specify the grid size of downsampling (defaults to 0.01)."
			<< std::endl;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_tracker");
	bool use_convex_hull = true;
	bool visualize_non_downsample = false;
	bool visualize_particles = true;
	bool use_fixed = false;

	double downsampling_grid_size = 0.01;

	if (pcl::console::find_argument(argc, argv, "-C") > 0)
		use_convex_hull = false;
	if (pcl::console::find_argument(argc, argv, "-D") > 0)
		visualize_non_downsample = true;
	if (pcl::console::find_argument(argc, argv, "-P") > 0)
		visualize_particles = false;
	if (pcl::console::find_argument(argc, argv, "-fixed") > 0)
		use_fixed = true;
	pcl::console::parse_argument(argc, argv, "-d", downsampling_grid_size);
	if (argc < 2) {
		usage(argv);
		exit(1);
	}

	std::string device_id = std::string(argv[1]);

	if (device_id == "--help" || device_id == "-h") {
		usage(argv);
		exit(1);
	}

	// open kinect
	OpenNISegmentTracking<pcl::PointXYZRGBA> v(device_id, 8,
			downsampling_grid_size, use_convex_hull, visualize_non_downsample,
			visualize_particles, use_fixed, 15);
	v.run();
}



