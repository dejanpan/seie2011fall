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

// pcl typedefs
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::PointXYZRGBNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudNormals;
typedef PointCloudNormals::Ptr PointCloudNormalsPtr;
typedef PointCloudNormals::ConstPtr PointCloudNormalsConstPtr;

void restoreCVMatFromPointCloud(PointCloudConstPtr cloud_in, cv::Mat & restored_image)
{
	restored_image = cv::Mat(cloud_in->height, cloud_in->width, CV_8UC3);
	for(uint rows = 0; rows < cloud_in->height; rows++){
		for (uint cols = 0; cols < cloud_in->width; ++cols) {
//			restored_image.at<uint8_t>(rows, cols) = cloud_in->at(cols, rows).r;
			restored_image.at<cv::Vec3b>(rows, cols)[0] = cloud_in->at(cols, rows).b;
			restored_image.at<cv::Vec3b>(rows, cols)[1] = cloud_in->at(cols, rows).g;
			restored_image.at<cv::Vec3b>(rows, cols)[2] = cloud_in->at(cols, rows).r;
		}
	}
}

int
 main (int argc, char** argv)
{
//	const cv::Mat input = cv::imread(argv[1], 0); //Load as grayscale
//
//	cv::SiftFeatureDetector detector;
//	std::vector<cv::KeyPoint> keypoints;
//	detector.detect(input, keypoints);
//
//	// Add results to image and save.
//	cv::Mat output;
//	cv::drawKeypoints(input, keypoints, output);
//	cv::imwrite("sift_result.jpg", output);


	PointCloudPtr cloud_in (new PointCloud);
	PointCloudPtr cloud_out (new PointCloud);

	//Fill in the cloud data
	pcl::PCDReader reader;
	reader.read (argv[1], *cloud_in);
	reader.read (argv[2], *cloud_out);

	cv::Mat cloud_in_image;
	restoreCVMatFromPointCloud(cloud_in, cloud_in_image);
	cv::Mat cloud_in_greyscale;
	cvtColor(cloud_in_image, cloud_in_greyscale, CV_RGB2GRAY);
	cv::imwrite("pointcloud_out.jpg", cloud_in_greyscale);

  return 0;
}
