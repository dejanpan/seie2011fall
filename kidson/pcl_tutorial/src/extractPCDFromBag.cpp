#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/transforms.h"

//rosbag stuff:
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "pcl_tutorial/featureMatch.h"
#include "pcl_tutorial/match.h"

int main (int argc, char** argv)
{
	if (argc != 2)
	{
	  std::cerr << "please provide filename" << std::endl;
	  exit(0);
	}
  	pcl::PCDWriter writer;
  	std::stringstream filename;

	rosbag::Bag bag;
	bag.open(argv[1], rosbag::bagmode::Read);
	std::vector<std::string> topics;
	topics.push_back(std::string("/camera/rgb/points"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));

	int i = 1;
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		sensor_msgs::PointCloud2::Ptr cloud = m.instantiate<sensor_msgs::PointCloud2>();
		ROS_INFO_STREAM("Writing cloud " << i << " to file");
		filename << "cloud" << i << ".pcd";
		writer.write (filename.str(), *cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
		filename.str("");
		i++;
	}
	bag.close();
}
