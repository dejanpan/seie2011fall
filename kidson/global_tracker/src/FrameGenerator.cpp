/*
 * FrameGenerator.cpp
 *
 *  Created on: Jul 27, 2011
 *      Author: engelhar
 */

#include "FrameGenerator.h"




//#define USE_SIFT_FOR_NODE_COMPS


using namespace sensor_msgs;
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>

namespace enc = sensor_msgs::image_encodings;

#include <pcl/filters/statistical_outlier_removal.h>

bool stronger(cv::KeyPoint a, cv::KeyPoint b)
{
  return a.response > b.response;
}


void FrameGenerator::printTiming(){

  double secs = ticks/cv::getTickFrequency();
  double secs_per_check = secs*1.0/nodes_cnt;

  ROS_ERROR("TIMING for generations: %f s for %i comps (%f s each)", secs, nodes_cnt, secs_per_check);


}

FrameGenerator::FrameGenerator(string calonder_file, bool create_dense_cloud, int min_strength, int point_cnt){


  // load database (SIFT only)
  //home/engelhar/Desktop/odu_files/images.tree",
    //                                           "/home/engelhar/Desktop/odu_files/images.weights"


  char* tree_file = "/opt/ros/diamondback/stacks/vslam/vocabulary_tree/holidays.tree";
  char* weights = "/opt/ros/diamondback/stacks/vslam/vocabulary_tree/holidays.weights";
  extractor = new cv::CalonderDescriptorExtractor<float>(calonder_file);


//  char* tree_file = "/home/engelhar/Desktop/odu_files/images.tree";
//  char* weights = "/home/engelhar/Desktop/odu_files/images.weights";
//  extractor = new cv::SiftDescriptorExtractor();


  ROS_INFO("load db");
  tree.load(tree_file);
  ROS_INFO("loaded tree with desc len of %i and %i words", tree.dimension(), tree.words());


  ROS_INFO("loaded tree with %i words", tree.words());

  vt_db = new vt::Database(tree.words());
  vt_db->loadWeights(weights);

  ROS_INFO("db is loaded");


  next_node_id = 0;


  detector = new cv::GridAdaptedFeatureDetector(new cv::FastFeatureDetector(10), 1000);
//



  ticks = nodes_cnt = 0;

  max_point_cnt = point_cnt;
  // detector = new cv::GridAdaptedFeatureDetector(new cv::FastFeatureDetector(min_strength), point_cnt, 640/20,480/20);
  // detector = new cv::FastFeatureDetector(min_strength); // Alternative from frame_proc
  this->create_dense_cloud = create_dense_cloud;

  has_groundtruth = false;
 }

void FrameGenerator::createNode(Node& node, const ImageConstPtr& img_rgb, const ImageConstPtr& img_depth,image_geometry::PinholeCameraModel cam_model){


  int64 start = cv::getTickCount();


  node.node_id = next_node_id++;
  node.stamp = img_rgb->header.stamp;

  node.cam_model = cam_model;

  //cout << "header stamp: " << node.stamp << endl;

  if (this->has_groundtruth)
  {
    stamped_trafo st;
    bool trafo_found = ground_truth.getTrafoAt(node.stamp, st);



    if (trafo_found){
      node.ground_truth_pose = st.trafo;
      node.ground_truth_valid = true;


      ROS_WARN("found pose for node %i with t= %f", node.node_id, node.stamp.toSec());
      cout << node.ground_truth_pose << endl;


      //      node.gt_pos_xy.x = st.trafo(0,3);
      //      node.gt_pos_xy.y = st.trafo(1,3);

      // node one is fixed in optimizer,
      // this initialization moves mocap and g2o in the same frame
      //      if (node.node_id == 0)
      //        node.pose = node.ground_truth_pose;
    }
    // cout << "gt: diff " << (st.time-node.stamp) << endl;
  }


  createFrame(node.frame, node.words, img_rgb, img_depth,cam_model);

  node.depth = cv_bridge::toCvCopy(img_depth,std::string("32FC1"))->image;


  if (create_dense_cloud)
    generatePointCloud(node);



  //  ros::Time  end = ros::Time::now();
  //  ROS_ERROR("######## time for cloud creation: %i ms", (int) (end-start).toNSec()/1000/1000);

  //  start = ros::Time::now();
  //  float voxel_size = 0.03;
  //
  //  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  //  vox_grid.setInputCloud (frame.dense_pointcloud.makeShared());
  //  vox_grid.setLeafSize (voxel_size,voxel_size,voxel_size);
  //  vox_grid.filter (frame.dense_pointcloud);

  // beautifies map, but takes some time
  //  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  //  sor.setInputCloud (frame.dense_pointcloud.makeShared());
  //  sor.setMeanK (50);
  //  sor.setStddevMulThresh (1.0);
  //  sor.filter(frame.dense_pointcloud);

  //  end = ros::Time::now();
  //  ROS_INFO("time for grid: %i ms",(int) (end-start).toNSec()/1000/1000);

  // TODO: set depth of removed pixels to nan


  int64 end = cv::getTickCount();

  ticks += (end-start);
  nodes_cnt++;


}

void FrameGenerator::createFrame(frame_common::Frame& frame, vt::Document& words, const ImageConstPtr& img_rgb, const ImageConstPtr& img_depth, image_geometry::PinholeCameraModel cam_model){


//  // same parameters as used in kidnapped_robot
//  detector->detect(frame.img, frame.kpts);
//  calonder_extractor->compute(frame.img, frame.kpts, frame.dtors);
//  words = vt::Document(frame.dtors.rows);
//  for (int i = 0; i < frame.dtors.rows; ++i){
//    words[i] =  place_db_->getVocTree()->quantize(frame.dtors.row(i)); }
//  sort(words.begin(), words.end());
//  return;




  //  ROS_INFO("createFrame 1");

  frame.img  =  cv_bridge::toCvCopy(img_rgb,enc::BGR8)->image;
  cv::Mat depth = cv_bridge::toCvCopy(img_depth,std::string("32FC1"))->image;
  cv::Mat mask = cv::Mat(480,640, CV_8UC1);

  for (uint i=0; i<640; i++)
    for (uint j=0; j<480;j++)
    {
      float d = depth.at<float>(j,i);
      if (! (d==d) || d < 0.1 || d > 100)
        mask.at<uchar>(j,i) = 0;
      else
        mask.at<uchar>(j,i) = 1;
    }

  //  cv::namedWindow("mask");
  //  cv::imshow("mask", mask);
  //  cv::namedWindow("frame.img");
  //  cv::imshow("frame.img", frame.img);
  //  cv::waitKey(100);

  //  ROS_INFO("createFrame 2");
  frame.kpts.clear();
//  detector->detect(frame.img, frame.kpts, mask);
  detector->detect(frame.img, frame.kpts);


  //  ROS_INFO("createFrame 3");
  uint kpts_cnt = frame.kpts.size();

//  ROS_INFO("found %i features, but only using the %i best", kpts_cnt, max_point_cnt);
//  if ( kpts_cnt > max_point_cnt ){
//    sort(frame.kpts.begin(), frame.kpts.end(), stronger);
//    frame.kpts.resize(max_point_cnt);
//  }

  kpts_cnt = frame.kpts.size();


//  bool valid[kpts_cnt];
//  float min_dist_sq = 5*5; // check for max reprojection error for the pose_estimator
//
//  for (uint i=0; i < kpts_cnt-1; ++i) valid[i] = true;
//  vector<cv::KeyPoint> copy = frame.kpts;
//  frame.kpts.clear();
//  frame.kpts.reserve(kpts_cnt);
//  for (uint i=0; i < kpts_cnt-1; ++i){
//
//    // i has already lost a comparison
//    if (!valid[i]) continue;
//
//    cv::KeyPoint kp = copy[i];
//    for (uint j= i+1; j< kpts_cnt; ++j)
//    {
//      double dx = kp.pt.x - frame.kpts[j].pt.x;
//      double dy = kp.pt.y - frame.kpts[j].pt.y;
//
//      double d = dx*dx+dy*dy;
//
//      // points are too close, remove weaker point
//      if (d < min_dist_sq){
//        // ROS_INFO("px dist: %f", sqrt(d));
//        if (kp.response > frame.kpts[j].response)
//          valid[j] = false;
//        else {
//          valid[i] = false;
//          break;
//        }
//      }
//    }
//
//    if (valid[i])
//      frame.kpts.push_back(kp);
//  }


  // compute calonder descriptors and quantize them
  extractor->compute(frame.img, frame.kpts, frame.dtors);

  // quantize extracted descriptors
  words = vt::Document(frame.dtors.rows);
  for (int i = 0; i < frame.dtors.rows; ++i){
     words[i] =  tree.quantize(frame.dtors.row(i)); //place_db_->getVocTree()


//    // creare EigenMatrix:
//    Feature e;
////    ROS_INFO("length of desc vector: %i", frame.dtors.cols);
//    for (int j=0; j<frame.dtors.cols; j++){
//        e(0,j) = frame.dtors.at<float>(j,i);
//    }
//
//    words[i] =  tree.quantize(e);


  }

  sort(words.begin(), words.end());


  //  cv::Mat foo = frame.img.clone();
  //  for (uint i=0; i<copy.size(); ++i){
  //      cv::circle(foo,copy[i].pt, 3, valid[i]?CV_RGB(0,255,0):CV_RGB(255,0,0), 1);
  //  }
  //  cv::namedWindow("removed features",1);
  //  cv::imshow("removed features", foo);
  //  cv::waitKey(10);



//  ROS_INFO("Using %i features, (found %i points)", (int) frame.kpts.size(), (int) kpts_cnt );

  /*
   * Fuer die Node-Vergleiche werden SIFT-Descriptoren eingesetzt
   */
#ifdef USE_SIFT_FOR_NODE_COMPS
  sift_extractor->compute(frame.img, frame.kpts, frame.dtors);
#endif



  // ros::Time e2 = ros::Time::now();


  frame.pts.resize(frame.kpts.size());
  frame.goodPts.assign(frame.kpts.size(),false);
  frame.disps.assign(frame.kpts.size(),10);

  frame_common::CamParams cam_params;
  cam_params.cx = cam_model.cx();
  cam_params.cy = cam_model.cy();
  cam_params.fx = cam_model.fx();
  cam_params.fy = cam_model.fy();
  cam_params.tx = 0.075; // reverse engineered... same value as in openni_node




  frame.setCamParams(cam_params);

  uint good_cnt = 0;

  // http://publib.boulder.ibm.com/infocenter/comphelp/v8v101/index.jsp?topic=%2Fcom.ibm.xlcpp8a.doc%2Fcompiler%2Fref%2Fruompplp.htm
  //  #pragma omp parallel for shared(depth, frame.kpts)
  // TODO: activate pragma

  int w = 3;
  uint nkpts = frame.kpts.size();
  for(uint i=0; i<nkpts; ++i){
    int px_x = frame.kpts[i].pt.x;
    int px_y = frame.kpts[i].pt.y;

    //    float d_0 = depth.at<float>(px_y,px_x);

    float dist = 100000;
    for (int x = px_x-w; x <= px_x+w; ++x)
      for (int y = px_y-w; y <= px_y+w; ++y){
        if (x<0 || y<0 || x>=640 || y >= 480)
          continue;
        float d  =  depth.at<float>(y,x);
        if (d<dist)
          dist = d;
      }


    cv::Point3d ray = cam_model.projectPixelTo3dRay(cv::Point(px_x,px_y))*dist;

    if (dist!=dist) // nan
    {
      ROS_ERROR("FAIL");
      frame.goodPts[i] = false;
    } else {
      frame.goodPts[i] = true;

      good_cnt++;

      frame.pts[i](0) = ray.x;
      frame.pts[i](1) = ray.y;
      frame.pts[i](2) = ray.z;
      frame.pts[i](3) = 1;

      float disp = cam_params.fx * cam_params.tx / dist;
      frame.disps[i] = disp;

      // ROS_INFO("disp: %f",disp);

    }
  }


  //  ROS_INFO("found %i valid points of %i (%f %%)",good_cnt, nkpts, good_cnt*100.0/nkpts );
  //
  //
  //
  //    for (uint i=0; i<frame.kpts.size(); ++i){
  //      cv::circle(frame.img,frame.kpts[i].pt, 3, CV_RGB(0,255,0), 1);
  //    }
  //
  //
  //    cv::namedWindow("rgb",1);
  //    cv::imshow("rgb", frame.img);
  //    cv::waitKey(10);

}

void FrameGenerator::generatePointCloud(Node& node, int step){


  // cloud was already computed
  if (node.has_dense_pointcloud) {
    return;
  }



  node.has_dense_pointcloud = true;
  node.frame.dense_pointcloud.points.reserve(640*480);


  //  ros::Time start = ros::Time::now();

  node.frame.dense_pointcloud.is_dense = false;

  //#pragma omp parallel for shared( node. )

  for (int i=0; i<640; i+=step)
    for (int j=0; j<480;j+=step)
    {
      float d = node.depth.at<float>(j,i);
      if (! (d==d) || d < 0.1 || d > 100)  continue;

      cv::Vec3b colors = node.frame.img.at<cv::Vec3b>(j,i);
      int32_t rgb = ( colors(2) << 16) | (colors(1) << 8) | (colors(0) );
      cv::Point3d ray = node.cam_model.projectPixelTo3dRay(cv::Point(i,j))*d;
      pcl::PointXYZRGB p3d;

      p3d.rgb = *(float *)(&rgb);
      p3d.x = ray.x;  // (i - cam_params.cx) / cam_params.fx * d;
      p3d.y = ray.y;  // (j - cam_params.cy) / cam_params.fy * d;
      p3d.z = ray.z;  // d;
      assert(p3d.x == p3d.x);
      ROS_ASSERT(abs(p3d.x) < 10);
      ROS_ASSERT(abs(p3d.y) < 10);
      ROS_ASSERT(abs(p3d.z) < 10);

      node.frame.dense_pointcloud.points.push_back(p3d);
    }

  //  ROS_INFO("created pointcloud with %i points", int(node.frame.dense_pointcloud.points.size()));

}
