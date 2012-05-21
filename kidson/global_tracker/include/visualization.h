/*
 * visualization.h
 *
 *  Created on: Jul 24, 2011
 *      Author: engelhar
 */

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/ImageMarker.h>
#include "FrameGenerator.h"


typedef map<int,vector<int> > match_list;



void readMatches(match_list& ml, string file);


void visualizeOdometry_GT(node_map& nodes, FrameGenerator& frame_gen, bool estimate = false);


void sendMarkers(ros::Publisher pub_marker, ros::Publisher pub_marker_array , node_map& nodes);

void sendClouds(ros::NodeHandle& nh, node_map& nodes, map<int,ros::Publisher>& publishers);

void sendClouds_simple(ros::Publisher pub, node_map& nodes);

void drawColoredMatches(const Node& n1, const Node& n2, const vector<cv::DMatch>&matches,  cv::Mat& res);

#endif /* VISUALIZATION_H_ */
