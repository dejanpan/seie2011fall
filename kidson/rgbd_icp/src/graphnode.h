/*
 * graphnode.h
 *
 *  Created on: Feb 13, 2012
 *      Author: ross
 */

#ifndef GRAPHNODE_CPP_
#define GRAPHNODE_CPP_

#include <ros/ros.h>

struct graphnode
{
	int id;
	sensor_msgs::PointCloud2 pointCloud;
};

#endif /* GRAPHNODE_CPP_ */
