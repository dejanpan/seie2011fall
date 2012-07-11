/*
 * joint_optimization.cpp
 *
 *  Created on: 11.07.2012
 *      Author: ross
 */

//sift gpu
#include "sift_gpu_wrapper.h"

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>

int
 main (int argc, char** argv)
{
	const cv::Mat input = cv::imread(argv[1], 0); //Load as grayscale

	cv::SiftFeatureDetector detector;
	std::vector<cv::KeyPoint> keypoints;
	detector.detect(input, keypoints);

	// Add results to image and save.
	cv::Mat output;
	cv::drawKeypoints(input, keypoints, output);
	cv::imwrite("sift_result.jpg", output);

//
//	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
//	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
//
//	  //Fill in the cloud data
//	  pcl::PCDReader reader;
//	  reader.read (argv[1], *cloud_in);
//	  reader.read (argv[2], *cloud_out);

	  return 0;
}
