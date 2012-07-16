/*
 * joint_optimization.cpp
 *
 *  Created on: 11.07.2012
 *      Author: ross
 */

//local files
#include "sift_gpu_wrapper.h"
#include "parameter_server.h"
#include "ransac_transformation.h"

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

void projectFeaturesTo3D(std::vector<cv::KeyPoint>& feature_locations_2d, std::vector<Eigen::Vector4f> & feature_locations_3d, PointCloudConstPtr point_cloud)
{
	int index = -1;
	for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
		++index;

		cv::Point2f p2d = feature_locations_2d[i].pt;
		PointType p3d = point_cloud->at((int) p2d.x,(int) p2d.y);

		// Check for invalid measurements
		if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z))
		{
			ROS_DEBUG("Feature %d has been extracted at NaN depth. Omitting", i);
			feature_locations_2d.erase(feature_locations_2d.begin()+i);
			continue;
		}

		feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
		//featuresUsed.push_back(index);  //save id for constructing the descriptor matrix
		i++; //Only increment if no element is removed from vector
	}
}

int
 main (int argc, char** argv)
{
	PointCloudPtr source_cloud (new PointCloud);
	PointCloudPtr target_cloud (new PointCloud);

	pcl::PCDReader reader;
	reader.read (argv[1], *source_cloud);
	reader.read (argv[2], *target_cloud);

	cv::Mat source_image;
	cv::Mat target_image;
	cv::Mat source_image_greyscale;
	cv::Mat target_image_greyscale;
	// get image from pointcloud
	restoreCVMatFromPointCloud(source_cloud, source_image);
	restoreCVMatFromPointCloud(target_cloud, target_image);
	// convert to black and white
	cvtColor(source_image, source_image_greyscale, CV_RGB2GRAY);
	cvtColor(target_image, target_image_greyscale, CV_RGB2GRAY);

	//detect sift features
	cv::SiftFeatureDetector detector;
	std::vector<cv::KeyPoint> source_keypoints, target_keypoints;
	detector.detect(source_image_greyscale, source_keypoints);
	detector.detect(target_image_greyscale, target_keypoints);

	cv::Mat output;
	cv::drawKeypoints(source_image_greyscale, source_keypoints, output);
	cv::imwrite("sift_result.jpg", output);

	// get sift descriptors
	cv::SiftDescriptorExtractor extractor;
	cv::Mat source_descriptors, target_descriptors;
	extractor.compute( source_image_greyscale, source_keypoints, source_descriptors );
	extractor.compute( target_image_greyscale, target_keypoints, target_descriptors );

	// project sift descriptors to 3d
	std::vector<Eigen::Vector4f> source_features_3d, target_features_3d;
	projectFeaturesTo3D(source_keypoints, source_features_3d, source_cloud);
	projectFeaturesTo3D(target_keypoints, target_features_3d, target_cloud);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	cv::FlannBasedMatcher matcher;
	std::vector< cv::DMatch > matches;
	matcher.match( source_descriptors, target_descriptors, matches );

	// Outlier detection
	// ##### OPENCV CODE- USE RANSAC FOR OUTLIER DETECTION #######
//	double max_dist = 0; double min_dist = 100;
//
//	//-- Quick calculation of max and min distances between keypoints
//	for( int i = 0; i < source_descriptors.rows; i++ )
//	{ double dist = matches[i].distance;
//	if( dist < min_dist ) min_dist = dist;
//	if( dist > max_dist ) max_dist = dist;
//	}
//
//	printf("-- Max dist : %f \n", max_dist );
//	printf("-- Min dist : %f \n", min_dist );
//
//	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
//	//-- PS.- radiusMatch can also be used here.
//	std::vector< cv::DMatch > good_matches;
//
//	for( int i = 0; i < source_descriptors.rows; i++ ){
//		if( matches[i].distance < 2*min_dist )
//			good_matches.push_back( matches[i]);
//	}
//	for( uint i = 0; i < good_matches.size(); i++ ){
//		printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
//	}

	//-- Draw only "good" matches
	cv::Mat img_matches;
	drawMatches( source_image, source_keypoints, target_image, target_keypoints,
			   matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
			   std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	//-- Show detected matches
	imshow( "Good Matches", img_matches );


	cv::waitKey(0);
  return 0;
}
