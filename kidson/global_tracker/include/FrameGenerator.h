/*
 * FrameGenerator.h
 *
 *  Creates a frame from a kinect color-depth pair.
 *
 *  Slightly inspired (aka completely stolen) from ros.org: frame_common::frameProc
 *
 *  used to generate frames which can be used by the posest stack
 *
 *  Created on: Jul 27, 2011
 *      Author: engelhar
 */

#ifndef FRAMEGENERATOR_H_
#define FRAMEGENERATOR_H_

#include <kidnapped_robot/place_database.h>
#include <frame_common/frame.h>
#include <frame_common/stereo.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "node_definitions.h"
#include <image_geometry/stereo_camera_model.h>




struct FrameGenerator {

 cv::Ptr<cv::FeatureDetector> detector;
// cv::Ptr<cv::DescriptorExtractor> sift_extractor;
// cv::Ptr<cv::DescriptorExtractor> calonder_extractor;

 cv::Ptr<cv::DescriptorExtractor> extractor;

 // for SIFT:
// typedef Eigen::Matrix<float, 1, 128> Feature;
//vt::VocabularyTree<Feature> tree;
 vt::GenericTree tree;

 vt::Database* vt_db;

 ros::Publisher pub;

 bool create_dense_cloud;
 // kidnapped_robot::PlaceDatabase* place_db_;

 uint next_node_id;
 uint max_point_cnt;

 Ground_truth ground_truth;
 bool has_groundtruth;


 int64 ticks;
 int nodes_cnt;

 void printTiming();

 FrameGenerator(string calonder_file, bool create_dense_cloud, int min_strength, int point_cnt);

 // calls createFrame and sets node_id
 void createNode(Node& node, const sensor_msgs::ImageConstPtr& img_rgb,
                 const sensor_msgs::ImageConstPtr& img_depth,
                 image_geometry::PinholeCameraModel cam_model);

 void generatePointCloud(Node& node, int skip = 3);


 private:
 void createFrame(frame_common::Frame& frame, vt::Document& words, const sensor_msgs::ImageConstPtr& img_rgb,
                  const sensor_msgs::ImageConstPtr& img_depth,
                  image_geometry::PinholeCameraModel cam_model);

};



#endif /* FRAMEGENERATOR_H_ */
