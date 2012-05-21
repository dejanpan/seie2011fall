#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/transforms.h"

//rosbag stuff:
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "pcl_tutorial/featureMatch.h"
#include "pcl_tutorial/match.h"

//opencv
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

sensor_msgs::CvBridge g_bridge;

int main(int argc, char** argv)
{
	if (argc != 2) {
		std::cerr << "please provide filename" << std::endl;
		exit(0);
	}
	pcl::PCDWriter writer;
	std::stringstream filename;

	rosbag::Bag bag;
	bag.open(argv[1], rosbag::bagmode::Read);
	std::vector<std::string> topics;
	topics.push_back(std::string("/camera/rgb/image_color"));
	topics.push_back(std::string("/camera/rgb/points"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));

	int imageNum = 1;
	int cloudNum = 1;
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		//write cloud to file
		sensor_msgs::PointCloud2::Ptr cloud = m.instantiate<sensor_msgs::PointCloud2>();
		if(cloud)
		{
			ROS_INFO_STREAM("Writing cloud " << cloudNum << " to file");
			filename.str("");
			filename << "cloud" << cloudNum << ".pcd";
			writer.write (filename.str(), *cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
			cloudNum++;
		}

		//write image to file
		sensor_msgs::ImagePtr image = m.instantiate<sensor_msgs::Image>();
		if(image)
		{
			if (g_bridge.fromImage(*image, "bgr8"))
			{
				IplImage *image = g_bridge.toIpl();
				if (image)
				{
					ROS_INFO_STREAM("Writing image " << imageNum << " to file");
					//std::string filename = (g_format % g_count % "jpg").str();
					filename.str("");
					filename << "image" << imageNum << ".bmp";
					cvSaveImage(filename.str().c_str(), image);
					//filename = (g_format % g_count % "ini").str();
					//camera_calibration_parsers::writeCalibration(filename, "camera", *info);
					imageNum++;
				}
				else
				{
					ROS_WARN("Couldn't save image, no data!");
				}
			}
		}
	}
	bag.close();
}
