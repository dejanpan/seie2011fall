/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "node.h"
#include <cmath>
#include <ctime>
#include <Eigen/Geometry>
#include "pcl/ros/conversions.h"
#include "pcl/io/pcd_io.h"
#include <pcl/common/transformation_from_correspondences.h>
//#include <opencv2/highgui/highgui.hpp>
#include <qtconcurrentrun.h>
#include <QtConcurrentMap>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

// joint optimizer
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/transformation_estimation_joint_optimize.h>
#include <pcl/registration/icp_joint_optimize.h>

#ifdef USE_SIFT_GPU
#include "sift_gpu_wrapper.h"
#endif

//#include <math.h>
#include <fstream>
#ifdef USE_ICP_BIN
#include "gicp-fallback.h"
#endif

#ifdef USE_ICP_CODE
#include "../gicp/transform.h"
#endif

//#include <iostream>
#include <Eigen/StdVector>
#include "misc.h"
#include <pcl/filters/voxel_grid.h>
#include <opencv/highgui.h>

Node::Node(const cv::Mat& visual, 
           const cv::Mat& depth,
           const cv::Mat& detection_mask,
           const sensor_msgs::CameraInfoConstPtr& cam_info, 
           std_msgs::Header depth_header,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor)
: id_(0), 
  pc_col(new pointcloud_type()),
  flannIndex(NULL),
  base2points_(tf::Transform::getIdentity(), depth_header.stamp, ParameterServer::instance()->get<std::string>("base_frame_name"), depth_header.frame_id),
  ground_truth_transform_(tf::Transform::getIdentity(), depth_header.stamp, ParameterServer::instance()->get<std::string>("ground_truth_frame_name"), ParameterServer::instance()->get<std::string>("base_frame_name")),
  initial_node_matches_(0)
{
  ParameterServer* ps = ParameterServer::instance();
  pc_col->header = depth_header;
#ifdef USE_ICP_CODE
  gicp_initialized = false;
#endif
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

#ifdef USE_SIFT_GPU
  std::vector<float> descriptors;
  if(ps->get<std::string>("feature_detector_type") == "SIFTGPU"){
    SiftGPUWrapper* siftgpu = SiftGPUWrapper::getInstance();
    siftgpu->detect(visual, feature_locations_2d_, descriptors);
    ROS_FATAL_COND(descriptors.size()==0, "Can't run SiftGPU");
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature Detection and Descriptor Extraction runtime: "<< elapsed <<" s");
  } else 
#endif
  {
    ROS_FATAL_COND(detector.empty(), "No valid detector!");
    detector->detect( visual, feature_locations_2d_, detection_mask);// fill 2d locations
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature Detection runtime: "<< elapsed <<" s");
  }

  // project pixels to 3dPositions and create search structures for the gicp
#ifdef USE_SIFT_GPU
  if(ps->get<std::string>("feature_detector_type") == "SIFTGPU"){
    projectTo3DSiftGPU(feature_locations_2d_, feature_locations_3d_, depth, cam_info, descriptors, feature_descriptors_); 
  }
  else
#endif
  {
    projectTo3D(feature_locations_2d_, feature_locations_3d_, depth, cam_info);
    struct timespec starttime2; clock_gettime(CLOCK_MONOTONIC, &starttime2);
    extractor->compute(visual, feature_locations_2d_, feature_descriptors_); //fill feature_descriptors_ with information 
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime2.tv_sec); elapsed += (finish.tv_nsec - starttime2.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature Extraction runtime: "<< elapsed <<" s");
  }
  assert(feature_locations_2d_.size() == feature_locations_3d_.size());
  ROS_INFO_NAMED("statistics", "Feature Count of Node:\t%d", (int)feature_locations_2d_.size());
  size_t max_keyp = ps->get<int>("max_keypoints");
  if(feature_locations_2d_.size() > max_keyp) {
    feature_locations_2d_.resize(max_keyp);
    feature_locations_3d_.resize(max_keyp);
    feature_descriptors_ = feature_descriptors_.rowRange(0,max_keyp-1);
  }

#ifdef USE_ICP_CODE
  createGICPStructures(); 
#endif
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");

}











Node::Node(const cv::Mat visual,
		   const cv::Mat visualColour,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor,
           pointcloud_type::Ptr point_cloud,
           const cv::Mat detection_mask)
: id_(0),
  pc_col(point_cloud),
  cameraImageColour(visualColour),
  flannIndex(NULL),
  base2points_(tf::Transform::getIdentity(), point_cloud->header.stamp,ParameterServer::instance()->get<std::string>("base_frame_name"), point_cloud->header.frame_id),
  ground_truth_transform_(tf::Transform::getIdentity(), point_cloud->header.stamp, ParameterServer::instance()->get<std::string>("ground_truth_frame_name"), ParameterServer::instance()->get<std::string>("base_frame_name")),
  initial_node_matches_(0)
{
  //cv::namedWindow("matches");
  ParameterServer* ps = ParameterServer::instance();

#ifdef USE_ICP_CODE
  gicp_initialized = false;
#endif
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

#ifdef USE_SIFT_GPU
  std::vector<float> descriptors;
  if(ps->get<std::string>("feature_detector_type") == "SIFTGPU"){
    SiftGPUWrapper* siftgpu = SiftGPUWrapper::getInstance();
    siftgpu->detect(visual, feature_locations_2d_, descriptors);
    ROS_FATAL_COND(descriptors.size() ==0, "Can't run SiftGPU");
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature Detection and Descriptor Extraction runtime: "<< elapsed <<" s");
  } else 
#endif
  {
    ROS_FATAL_COND(detector.empty(), "No valid detector!");
    detector->detect( visual, feature_locations_2d_, detection_mask);// fill 2d locations
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature Detection runtime: "<< elapsed <<" s");
  }

  // project pixels to 3dPositions and create search structures for the gicp
#ifdef USE_SIFT_GPU
  if(ps->get<std::string>("feature_detector_type") == "SIFTGPU"){
    // removes also unused descriptors from the descriptors matrix
    // build descriptor matrix and sets siftgpu_descriptors!
    projectTo3DSiftGPU(feature_locations_2d_, feature_locations_3d_, pc_col, descriptors, feature_descriptors_); //takes less than 0.01 sec
  }
  else
#endif
  {
    projectTo3D(feature_locations_2d_, feature_locations_3d_, pc_col); //takes less than 0.01 sec
    // projectTo3d need a dense cloud to use the points.at(px.x,px.y)-Call
    struct timespec starttime2; clock_gettime(CLOCK_MONOTONIC, &starttime2);
    extractor->compute(visual, feature_locations_2d_, feature_descriptors_); //fill feature_descriptors_ with information 
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime2.tv_sec); elapsed += (finish.tv_nsec - starttime2.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature Extraction runtime: "<< elapsed <<" s");
  }
  assert(feature_locations_2d_.size() == feature_locations_3d_.size());
  ROS_INFO_NAMED("statistics", "Feature Count of Node:\t%d", (int)feature_locations_2d_.size());
  size_t max_keyp = ps->get<int>("max_keypoints");
  if(feature_locations_2d_.size() > max_keyp) {
    feature_locations_2d_.resize(max_keyp);
    feature_locations_3d_.resize(max_keyp);
    feature_descriptors_ = feature_descriptors_.rowRange(0,max_keyp-1);
  }

#ifdef USE_ICP_CODE
  createGICPStructures(); 
#endif

  if((!ps->get<bool>("use_glwidget") ||
      !ps->get<bool>("use_gui")) &&
     !ps->get<bool>("store_pointclouds"))
  {
    ROS_WARN("Clearing out points");
    this->clearPointCloud();
  } else if(ps->get<double>("voxelfilter_size") > 0.0) {
    double vfs = ps->get<double>("voxelfilter_size");
    pcl::VoxelGrid<point_type> sor;
    sor.setLeafSize(vfs,vfs,vfs);
    pointcloud_type::ConstPtr const_cloud_ptr = boost::make_shared<pointcloud_type> (*pc_col);                                                                 
    sor.setInputCloud (const_cloud_ptr);
    sor.filter (*pc_col);
  }
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");

}

Node::~Node() {
    if (ParameterServer::instance()->get<std::string> ("feature_detector_type").compare("ORB") != 0) {
        if (flannIndex)
            delete flannIndex;
    }
}

void Node::setGroundTruthTransform(tf::StampedTransform gt){
    ground_truth_transform_ = gt;
}
void Node::setBase2PointsTransform(tf::StampedTransform b2p){
    base2points_ = b2p;
}
tf::StampedTransform Node::getGroundTruthTransform(){
    return ground_truth_transform_;
}
tf::StampedTransform Node::getBase2PointsTransform(){
    return base2points_;
}

void Node::publish(const char* frame, ros::Time timestamp, ros::Publisher pub){
    sensor_msgs::PointCloud2 cloudMessage;
    pcl::toROSMsg((*pc_col),cloudMessage);
    cloudMessage.header.frame_id = frame;
    cloudMessage.header.stamp = timestamp;
    pub.publish(cloudMessage);
    ROS_INFO("Pointcloud with id %i sent with frame %s", id_, frame);
}

#ifdef USE_ICP_CODE
bool Node::getRelativeTransformationTo_ICP_code(const Node* target_node,Eigen::Matrix4f& transformation,
    const Eigen::Matrix4f* initial_transformation){
  //std::clock_t starttime_icp = std::clock();
  dgc_transform_t initial;

  // use optional matrix as first guess in icp
  if (initial_transformation == NULL){
    gicpSetIdentity(initial); 
  }else {
    Eigen2GICP(*initial_transformation,initial);
  }


  dgc_transform_t final_trafo;
  dgc_transform_identity(final_trafo);


  assert(gicp_initialized && target_node->gicp_initialized);

  unsigned int iterations = target_node->gicp_point_set->AlignScan(this->gicp_point_set, initial, final_trafo, gicp_d_max);


  GICP2Eigen(final_trafo,transformation);

  //clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");

  return iterations < gicp_max_iterations;

}

# endif

#ifdef USE_ICP_BIN
bool Node::getRelativeTransformationTo_ICP_bin(const Node* target_node,
    Eigen::Matrix4f& transformation,
    const Eigen::Matrix4f* initial_transformation){
  std::clock_t starttime_icp = std::clock();

  bool converged;

  if (initial_transformation != NULL)
  {
    pointcloud_type pc2;
    pcl::transformPointCloud((*pc_col),pc2,*initial_transformation);
    converged = gicpfallback(pc2,target_node->(*pc_col), transformation);
  }
  else {
    converged = gicpfallback((*pc_col),target_node->(*pc_col), transformation); }

  // Paper
  // clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");

  return converged;
}
#endif

// build search structure for descriptor matching
void Node::buildFlannIndex() {
  if (ParameterServer::instance()->get<std::string> ("matcher_type") == "FLANN" 
      && ParameterServer::instance()->get<std::string> ("feature_extractor_type") != "ORB")
  {
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    //KDTreeIndexParams When passing an object of this type the index constructed will 
    //consist of a set of randomized kd-trees which will be searched in parallel.
    flannIndex = new cv::flann::Index(feature_descriptors_, cv::flann::KDTreeIndexParams(16));
    ROS_DEBUG("Built flannIndex (address %p) for Node %i", flannIndex, this->id_);
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
  }
}


#ifdef USE_ICP_CODE
void Node::createGICPStructures(unsigned int max_count){
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  gicp_point_set = new dgc::gicp::GICPPointSet();

  dgc::gicp::GICPPoint g_p;
  g_p.range = -1;
  for(int k = 0; k < 3; k++) {
    for(int l = 0; l < 3; l++) {
      g_p.C[k][l] = (k == l)?1:0;
    }
  }

  int step = 1;
  if ((*pc_col).points.size()>max_count)
    step = ceil((*pc_col).points.size()*1.0/max_count);

  int cnt = 0;
  for (unsigned int i=0; i<(*pc_col).points.size(); i++ ){
    point_type  p = (*pc_col).points.at(i);
    if (!(isnan(p.x) || isnan(p.y) || isnan(p.z))) {
      // add points to pointset for icp
      if (cnt++%step == 0){
        g_p.x=p.x;
        g_p.y=p.y;
        g_p.z=p.z;
        gicp_point_set->AppendPoint(g_p);
      }
    }
  }
  ROS_WARN("gicp_point_set.Size() %i", gicp_point_set->Size() );


  // build search structure for gicp:
  gicp_point_set->SetDebug(false);
  gicp_point_set->SetGICPEpsilon(gicp_epsilon);
  gicp_point_set->BuildKDTree();
  gicp_point_set->ComputeMatrices();
  gicp_point_set->SetMaxIterationInner(8); // as in test_gicp->cpp
  gicp_point_set->SetMaxIteration(gicp_max_iterations);
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");

  ROS_INFO_STREAM("time for creating the structure: " << ((std::clock()-starttime_gicp*1.0) / (double)CLOCKS_PER_SEC));
  ROS_INFO_STREAM("current: " << std::clock() << " " << "start_time: " << starttime_gicp);

  gicp_initialized = true;
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}
#endif

//TODO: This function seems to be resistant to parallelization probably due to knnSearch
int Node::findPairsFlann(const Node* other, std::vector<cv::DMatch>* matches) const {
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  assert(matches->size()==0);
  // number of neighbours found (two, to compare the best matches for distinctness
  const int k = 2;

  // number of neighbors found (has to be two, see l. 57)
  double sum_distances = 0.0;
  ParameterServer* ps = ParameterServer::instance();
  const int min_kp = ps->get<int> ("min_keypoints");

  //using siftgpu, if available and wanted
#ifdef USE_SIFT_GPU
  if (ps->get<std::string> ("matcher_type") == "SIFTGPU") {
    sum_distances = SiftGPUWrapper::getInstance()->match(siftgpu_descriptors, feature_descriptors_.rows, other->siftgpu_descriptors, other->feature_descriptors_.rows, matches);
  }
  else
#endif
  //using BruteForceMatcher for ORB features
  if (ps->get<std::string> ("matcher_type") == "BRUTEFORCE" || 
      ps->get<std::string> ("feature_extractor_type") == "ORB")
  {
    cv::Ptr<cv::DescriptorMatcher> matcher;
    std::string brute_force_type("BruteForce"); //L2 per default
    if(ps->get<std::string> ("feature_extractor_type") == "ORB"){
      brute_force_type.append("-HammingLUT");
    }
    matcher = cv::DescriptorMatcher::create(brute_force_type);
    std::vector< std::vector<cv::DMatch> > bruteForceMatches;
    matcher->knnMatch(feature_descriptors_, other->feature_descriptors_, bruteForceMatches, k);
    int dist_ratio_fac = 0.6;
    if ((int)bruteForceMatches.size() < min_kp) dist_ratio_fac = 1.0; //if necessary use possibly bad descriptors
    for (unsigned int i = 0; i < bruteForceMatches.size(); i++) {
        cv::DMatch m1 = bruteForceMatches[i][0];
        cv::DMatch m2 = bruteForceMatches[i][1];
        if (m1.distance < 0.6 * m2.distance) {//this check seems crucial to matching quality
            matches->push_back(m1);
            sum_distances += m1.distance;
        }
    }
    //matcher->match(feature_descriptors_, other->feature_descriptors_, *matches);
  } 
  else if (ps->get<std::string>("matcher_type") == "FLANN" && 
           ps->get<std::string>("feature_extractor_type") != "ORB")
  {
    if (other->flannIndex == NULL) {
        ROS_FATAL("Node %i in findPairsFlann: flann Index of Node %i was not initialized", this->id_, other->id_);
        return -1;
    }
    // compare
    // http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
    cv::Mat indices(feature_descriptors_.rows, k, CV_32S);
    cv::Mat dists(feature_descriptors_.rows, k, CV_32F);

    // get the best two neighbors
    other->flannIndex->knnSearch(feature_descriptors_, indices, dists, k, cv::flann::SearchParams(64));
    //64: The number of times the tree(s) in the index should be recursively traversed. A higher value for this parameter would give better search precision, but also take more time. If automatic configuration was used when the index was created, the number of checks required to achieve the specified precision was also computed, in which case this parameter is ignored.

    int* indices_ptr = indices.ptr<int> (0);
    float* dists_ptr = dists.ptr<float> (0);

    cv::DMatch match;
    int dist_ratio_fac = 0.6;
    if (indices.rows < min_kp) dist_ratio_fac = 1.0; //if necessary use possibly bad descriptors
    for (int i = 0; i < indices.rows; ++i) {
      if (dists_ptr[2 * i] < 0.6 * dists_ptr[2 * i + 1]) {
        match.queryIdx = i;
        match.trainIdx = indices_ptr[2 * i];
        match.distance = dists_ptr[2 * i];
        sum_distances += match.distance;

        assert(match.trainIdx < other->feature_descriptors_.rows);
        assert(match.queryIdx < feature_descriptors_.rows);

        matches->push_back(match);
      }
    }
  }
  else {
    ROS_FATAL_STREAM("Cannot match features:\nNo valid combination for " <<
                     "matcher_type ("           << ps->get<std::string>("matcher_type") << ") and " <<
                     "feature_extractor_type (" << ps->get<std::string>("feature_extractor_type") << ") chosen.");
  }

  ROS_INFO_NAMED("statistics", "count_matrix(%3d, %3d) =  %4d;",
                 this->id_+1, other->id_+1, (int)matches->size());
  ROS_INFO_NAMED("statistics", "dista_matrix(%3d, %3d) =  %f;",
                 this->id_+1, other->id_+1, sum_distances/ (float)matches->size());
  ROS_DEBUG_NAMED("statistics", "Feature Matches between Nodes %3d (%4d features) and %3d (%4d features):\t%4d",
                  this->id_, (int)this->feature_locations_2d_.size(),
                  other->id_, (int)other->feature_locations_2d_.size(),
                  (int)matches->size());

  //ROS_INFO("matches size: %i, rows: %i", (int) matches->size(), feature_descriptors_.rows);

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
  return matches->size();
}



#ifdef USE_SIFT_GPU
void Node::projectTo3DSiftGPU(std::vector<cv::KeyPoint>& feature_locations_2d,
                              std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                              const cv::Mat& depth,
                              const sensor_msgs::CameraInfoConstPtr& cam_info,
                              std::vector<float>& descriptors_in, cv::Mat& descriptors_out)
{

  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  double depth_scaling = ParameterServer::instance()->get<double>("depth_scaling_factor");
  float x,y;//temp point, 
  //principal point and focal lengths:
  float cx = cam_info->K[2]; //(cloud_msg->width >> 1) - 0.5f;
  float cy = cam_info->K[5]; //(cloud_msg->height >> 1) - 0.5f;
  float fx = 1.0f / cam_info->K[0]; 
  float fy = 1.0f / cam_info->K[4]; 
  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  std::list<int> featuresUsed;
  
  int index = -1;
  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    ++index;

    p2d = feature_locations_2d[i].pt;
    float Z = depth.at<float>(p2d.y, p2d.x) * depth_scaling;

    // Check for invalid measurements
    if (std::isnan (Z))
    {
      ROS_DEBUG("Feature %d has been extracted at NaN depth. Omitting", i);
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    x = (p2d.x - cx) * Z * fx;
    y = (p2d.y - cy) * Z * fy;

    feature_locations_3d.push_back(Eigen::Vector4f(x,y, Z, 1.0));
    featuresUsed.push_back(index);  //save id for constructing the descriptor matrix
    i++; //Only increment if no element is removed from vector
  }

  //create descriptor matrix
  int size = feature_locations_3d.size();
  descriptors_out = cv::Mat(size, 128, CV_32F);
  siftgpu_descriptors.resize(size * 128);
  for (int y = 0; y < size && featuresUsed.size() > 0; ++y) {
    int id = featuresUsed.front();
    featuresUsed.pop_front();

    for (int x = 0; x < 128; ++x) {
      descriptors_out.at<float>(y, x) = descriptors_in[id * 128 + x];
      siftgpu_descriptors[y * 128 + x] = descriptors_in[id * 128 + x];
    }
  }
  /*
  //create descriptor matrix
  int size = feature_locations_3d.size();
  descriptors_out = cv::Mat(size, 128, CV_32F);
  for (int y = 0; y < size && featuresUsed.size() > 0; ++y) {
    int id = featuresUsed.front();
    featuresUsed.pop_front();

    for (int x = 0; x < 128; ++x) {
      descriptors_out.at<float>(y, x) = descriptors_in[id * 128 + x];
    }
  }
  */

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void Node::projectTo3DSiftGPU(std::vector<cv::KeyPoint>& feature_locations_2d,
                              std::vector<Eigen::Vector4f, 
                              Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                              const pointcloud_type::Ptr point_cloud, 
                              std::vector<float>& descriptors_in, cv::Mat& descriptors_out)
{

  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  std::list<int> featuresUsed;

  int index = -1;
  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    ++index;

    p2d = feature_locations_2d[i].pt;
    point_type p3d = point_cloud->at((int) p2d.x,(int) p2d.y);

    // Check for invalid measurements
    if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z))
    {
      ROS_DEBUG("Feature %d has been extracted at NaN depth. Omitting", i);
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
    featuresUsed.push_back(index);  //save id for constructing the descriptor matrix
    i++; //Only increment if no element is removed from vector
  }

  //create descriptor matrix
  int size = feature_locations_3d.size();
  descriptors_out = cv::Mat(size, 128, CV_32F);
  siftgpu_descriptors.resize(size * 128);
  for (int y = 0; y < size && featuresUsed.size() > 0; ++y) {
    int id = featuresUsed.front();
    featuresUsed.pop_front();

    for (int x = 0; x < 128; ++x) {
      descriptors_out.at<float>(y, x) = descriptors_in[id * 128 + x];
      siftgpu_descriptors[y * 128 + x] = descriptors_in[id * 128 + x];
    }
  }

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}
#endif

void Node::projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
                       std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                       pointcloud_type::ConstPtr point_cloud)
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    p2d = feature_locations_2d[i].pt;
    if (p2d.x >= point_cloud->width || p2d.x < 0 ||
        p2d.y >= point_cloud->height || p2d.y < 0 ||
        std::isnan(p2d.x) || std::isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
      ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    point_type p3d = point_cloud->at((int) p2d.x,(int) p2d.y);

    // Check for invalid measurements
    if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z))
    {
      ROS_DEBUG("Feature %d has been extracted at NaN depth. Omitting", i);
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
    i++; //Only increment if no element is removed from vector
  }

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void Node::projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
                       std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                       const cv::Mat& depth,
                       const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  double depth_scaling = ParameterServer::instance()->get<double>("depth_scaling_factor");
  float x,y;//temp point, 
  //principal point and focal lengths:
  float cx = cam_info->K[2]; //(cloud_msg->width >> 1) - 0.5f;
  float cy = cam_info->K[5]; //(cloud_msg->height >> 1) - 0.5f;
  float fx = 1.0f / cam_info->K[0]; 
  float fy = 1.0f / cam_info->K[4]; 

  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    p2d = feature_locations_2d[i].pt;
    if (p2d.x >= depth.cols || p2d.x < 0 ||
        p2d.y >= depth.rows || p2d.y < 0 ||
        std::isnan(p2d.x) || std::isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
      ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    float Z = depth.at<float>(p2d.y, p2d.x) * depth_scaling;

    // Check for invalid measurements
    if (std::isnan (Z))
    {
      ROS_DEBUG("Feature %d has been extracted at NaN depth. Omitting", i);
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    x = (p2d.x - cx) * Z * fx;
    y = (p2d.y - cy) * Z * fy;

    feature_locations_3d.push_back(Eigen::Vector4f(x,y, Z, 1.0));
    i++; //Only increment if no element is removed from vector
  }

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}




void Node::computeInliersAndError(const std::vector<cv::DMatch>& matches,
                                  const Eigen::Matrix4f& transformation,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
                                  std::vector<cv::DMatch>& inliers, //output var
                                  double& mean_error,
                                  std::vector<double>& errors,
                                  double squaredMaxInlierDistInM) const{ //output var

  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  inliers.clear();
  errors.clear();

  std::vector<std::pair<float,int> > dists;
  std::vector<cv::DMatch> inliers_temp;

  assert(matches.size() > 0);
  mean_error = 0.0;
  for (unsigned int j = 0; j < matches.size(); j++){ //compute new error and inliers

    unsigned int this_id = matches[j].queryIdx;
    unsigned int earlier_id = matches[j].trainIdx;

    Eigen::Vector4f vec = (transformation * origins[this_id]) - earlier[earlier_id];

    double error = vec.dot(vec);

    if(error > squaredMaxInlierDistInM)
      continue; //ignore outliers
    if(!(error >= 0.0)){
      ROS_DEBUG_STREAM("Transformation for error !> 0: " << transformation);
      ROS_DEBUG_STREAM(error << " " << matches.size());
    }
    error = sqrt(error);
    dists.push_back(std::pair<float,int>(error,j));
    inliers_temp.push_back(matches[j]); //include inlier

    mean_error += error;
    errors.push_back(error);
  }

  if (inliers_temp.size()<3){ //at least the samples should be inliers
    ROS_WARN_COND(inliers_temp.size() > 3, "No inliers at all in %d matches!", (int)matches.size()); // only warn if this checks for all initial matches
    mean_error = 1e9;
  } else {
    mean_error /= inliers_temp.size();

    // sort inlier ascending according to their error
    sort(dists.begin(),dists.end());

    inliers.resize(inliers_temp.size());
    for (unsigned int i=0; i<inliers_temp.size(); i++){
      inliers[i] = matches[dists[i].second];
    }
  }
  if(!(mean_error>0)) ROS_DEBUG_STREAM("Transformation for mean error !> 0: " << transformation);
  if(!(mean_error>0)) ROS_DEBUG_STREAM(mean_error << " " << inliers_temp.size());
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");

}

template<class InputIterator>
Eigen::Matrix4f Node::getTransformFromMatches(const Node* earlier_node,
                                              InputIterator iter_begin,
                                              InputIterator iter_end,
                                              bool& valid, 
                                              float max_dist_m) const 
{
  pcl::TransformationFromCorrespondences tfc;
  valid = true;
  std::vector<Eigen::Vector3f> t, f;

  for ( ;iter_begin!=iter_end; iter_begin++) {
    int this_id    = iter_begin->queryIdx;
    int earlier_id = iter_begin->trainIdx;

    Eigen::Vector3f from(this->feature_locations_3d_[this_id][0],
                         this->feature_locations_3d_[this_id][1],
                         this->feature_locations_3d_[this_id][2]);
    Eigen::Vector3f  to (earlier_node->feature_locations_3d_[earlier_id][0],
                         earlier_node->feature_locations_3d_[earlier_id][1],
                         earlier_node->feature_locations_3d_[earlier_id][2]);
    if (max_dist_m > 0) {  //storing is only necessary, if max_dist is given
      f.push_back(from);
      t.push_back(to);    
    }
    tfc.add(from, to, 1.0/to(2));//the further, the less weight b/c of accuracy decay
  }


  // find smalles distance between a point and its neighbour in the same cloud
  // je groesser das dreieck aufgespannt ist, desto weniger fallen kleine positionsfehler der einzelnen
  // Punkte ist Gewicht!

  if (max_dist_m > 0)
  {  
    //float min_neighbour_dist = 1e6;
    Eigen::Matrix4f foo;

    valid = true;
    for (uint i=0; i<f.size(); i++)
    {
      float d_f = (f.at((i+1)%f.size())-f.at(i)).norm();
      float d_t = (t.at((i+1)%t.size())-t.at(i)).norm();

      if ( abs(d_f-d_t) > max_dist_m ) {
        valid = false;
        return Eigen::Matrix4f();
      }
    }
    //here one could signal that some samples are very close, but as the transformation is validated elsewhere we don't
    //if (min_neighbour_dist < 0.5) { ROS_INFO...}
  }
  // get relative movement from samples
  return tfc.getTransformation().matrix();
}


///Find transformation with largest support, RANSAC style.
///Return false if no transformation can be found
bool Node::getRelativeTransformationTo(const Node* earlier_node,
                                       std::vector<cv::DMatch>* initial_matches,
                                       Eigen::Matrix4f& resulting_transformation,
                                       float& rmse, 
                                       std::vector<cv::DMatch>& matches,
                                       int min_matches) const
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  assert(initial_matches != NULL);
  matches.clear();
  
  if(initial_matches->size() <= min_matches) {
	  ROS_INFO("Only %d feature matches between %d and %d (minimal: %i)",(int)initial_matches->size() , this->id_, earlier_node->id_, min_matches);
    return false;
  }
  else
  {
	  ROS_INFO("%d feature matches between %d and %d (minimal: %i)",(int)initial_matches->size() , this->id_, earlier_node->id_, min_matches);
  }

  //unsigned int min_inlier_threshold = int(initial_matches->size()*0.2);
  unsigned int min_inlier_threshold = (unsigned int) min_matches;
  std::vector<cv::DMatch> inlier; //holds those feature correspondences that support the transformation
  double inlier_error; //all squared errors
  srand((long)std::clock());
  
  // a point is an inlier if it's no more than max_dist_m m from its partner apart
  const float max_dist_m = ParameterServer::instance()->get<double>("max_dist_for_inliers");
  const int ransac_iterations = ParameterServer::instance()->get<int>("ransac_iterations");
  std::vector<double> dummy;

  // best values of all iterations (including invalids)
  double best_error = 1e6, best_error_invalid = 1e6;
  unsigned int best_inlier_invalid = 0, best_inlier_cnt = 0, valid_iterations = 0;

  Eigen::Matrix4f transformation;
  
  const unsigned int sample_size = 3;// chose this many randomly from the correspondences:
  for (int n_iter = 0; n_iter < ransac_iterations; n_iter++) {
    //generate a map of samples. Using a map solves the problem of drawing a sample more than once
    std::set<cv::DMatch> sample_matches;
    std::vector<cv::DMatch> sample_matches_vector;
    while(sample_matches.size() < sample_size){
      int id = rand() % initial_matches->size();
      sample_matches.insert(initial_matches->at(id));
      sample_matches_vector.push_back(initial_matches->at(id));
    }

    bool valid; // valid is false iff the sampled points clearly aren't inliers themself 
    transformation = getTransformFromMatches(earlier_node, sample_matches.begin(), sample_matches.end(),valid,max_dist_m);
    if (!valid) continue; // valid is false iff the sampled points aren't inliers themself 
    if(transformation!=transformation) continue; //Contains NaN
    
    //test whether samples are inliers (more strict than before)
    computeInliersAndError(sample_matches_vector, transformation, this->feature_locations_3d_, 
                           earlier_node->feature_locations_3d_, inlier, inlier_error,  /*output*/
                           dummy, max_dist_m*max_dist_m); 
    ROS_DEBUG("Transformation from and for %u samples results in an error of %f and %i inliers.", sample_size, inlier_error, (int)inlier.size());
    if(inlier_error > 1000) continue; //most possibly a false match in the samples
    computeInliersAndError(*initial_matches, transformation, this->feature_locations_3d_, 
                           earlier_node->feature_locations_3d_, inlier, inlier_error,  /*output*/
                           dummy, max_dist_m*max_dist_m);
    ROS_DEBUG("Transformation from %u samples results in an error of %f and %i inliers for all matches (%i).", sample_size, inlier_error, (int)inlier.size(), (int)initial_matches->size());

    // check also invalid iterations
    if (inlier.size() > best_inlier_invalid) {
      best_inlier_invalid = inlier.size();
      best_error_invalid = inlier_error;
    }
    // ROS_INFO("iteration %d  cnt: %d, best: %d,  error: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100);

    if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
      ROS_DEBUG("Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
      continue;
    }
    // ROS_INFO("Refining iteration from %i samples: all matches: %i, inliers: %i, inlier_error: %f", (int)sample_size, (int)initial_matches->size(), (int)inlier.size(), inlier_error);
    valid_iterations++;
    //if (inlier_error > 0) ROS_ERROR("size: %i", (int)dummy.size());
    assert(inlier_error>=0);

    //Performance hacks:
    ///Iterations with more than half of the initial_matches inlying, count twice
    if (inlier.size() > initial_matches->size()*0.5) n_iter+=10;
    ///Iterations with more than 80% of the initial_matches inlying, count threefold
    if (inlier.size() > initial_matches->size()*0.8) n_iter+=20;



    if (inlier_error < best_error) { //copy this to the result
      resulting_transformation = transformation;
      matches = inlier;
      assert(matches.size()>= min_inlier_threshold);
      best_inlier_cnt = inlier.size();
      //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
      rmse = inlier_error;
      best_error = inlier_error;
      // ROS_INFO("  new best iteration %d  cnt: %d, best_inlier: %d,  error: %.4f, bestError: %.4f",n_iter, inlier.size(), best_inlier_cnt, inlier_error, best_error);

    }else
    {
      // ROS_INFO("NO new best iteration %d  cnt: %d, best_inlier: %d,  error: %.4f, bestError: %.4f",n_iter, inlier.size(), best_inlier_cnt, inlier_error, best_error);
    }

    //int max_ndx = min((int) min_inlier_threshold,30); //? What is this 30?
    double new_inlier_error;

    transformation = getTransformFromMatches(earlier_node, matches.begin(), matches.end(), valid); // compute new trafo from all inliers:
    if(transformation!=transformation) continue; //Contains NaN
    computeInliersAndError(*initial_matches, transformation,
                           this->feature_locations_3d_, earlier_node->feature_locations_3d_,
                           inlier, new_inlier_error, dummy, max_dist_m*max_dist_m);
    ROS_DEBUG("Refined Transformation from all matches (%i) results in an error of %f and %i inliers for all matches.", (int)initial_matches->size(), inlier_error, (int)inlier.size());
    // ROS_INFO("asd recomputed: inliersize: %i, inlier error: %f", (int) inlier.size(),100*new_inlier_error);


    // check also invalid iterations
    if (inlier.size() > best_inlier_invalid)
    {
      best_inlier_invalid = inlier.size();
      best_error_invalid = inlier_error;
    }

    if(inlier.size() < min_inlier_threshold || new_inlier_error > max_dist_m){
      //inlier.size() < ((float)initial_matches->size())*min_inlier_ratio || 
      // ROS_INFO("Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
      continue;
    }
    // ROS_INFO("Refined iteration from %i samples: all matches %i, inliers: %i, new_inlier_error: %f", (int)sample_size, (int)initial_matches->size(), (int)inlier.size(), new_inlier_error);

    assert(new_inlier_error>=0);

    if (new_inlier_error < best_error) 
    {
      resulting_transformation = transformation;
      matches = inlier;
      assert(matches.size()>= min_inlier_threshold);
      //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
      rmse = new_inlier_error;
      best_error = new_inlier_error;
      // ROS_INFO("  improved: new best iteration %d  cnt: %d, best_inlier: %d,  error: %.2f, bestError: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100, best_error*100);
    }else
    {
      // ROS_INFO("improved: NO new best iteration %d  cnt: %d, best_inlier: %d,  error: %.2f, bestError: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100, best_error*100);
    }
  } //iterations
  ROS_INFO("%i good iterations (from %i), inlier pct %i, inlier cnt: %i, error: %.2f cm",valid_iterations, ransac_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size(),rmse*100);
  // ROS_INFO("best overall: inlier: %i, error: %.2f",best_inlier_invalid, best_error_invalid*100);

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
  return matches.size() >= min_inlier_threshold;
}


#ifdef USE_ICP_CODE
void Node::Eigen2GICP(const Eigen::Matrix4f& m, dgc_transform_t g_m){
  for(int i=0;i<4; i++)
    for(int j=0;j<4; j++)
      g_m[i][j] = m(i,j);

}
void Node::GICP2Eigen(const dgc_transform_t g_m, Eigen::Matrix4f& m){
  for(int i=0;i<4; i++)
    for(int j=0;j<4; j++)
      m(i,j) = g_m[i][j];
}

void Node::gicpSetIdentity(dgc_transform_t m){
  for(int i=0;i<4; i++)
    for(int j=0;j<4; j++)
      if (i==j)
        m[i][j] = 1;
      else
        m[i][j] = 0;
}
#endif


MatchingResult Node::matchNodePair(const Node* older_node, unsigned int min_matches){
  MatchingResult mr;
  if(initial_node_matches_ > ParameterServer::instance()->get<int>("max_connections")) return mr; //enough is enough
  //const unsigned int min_matches = (unsigned int) ParameterServer::instance()->get<int>("min_matches");// minimal number of feature correspondences to be a valid candidate for a link
  // struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  this->findPairsFlann(older_node, &mr.all_matches); 

  ROS_DEBUG("found %i inital matches",(int) mr.all_matches.size());
  if ((mr.all_matches.size() < min_matches)){
    ROS_INFO("Too few inliers: Adding no Edge between %i and %i. Only %i correspondences to begin with.",
        older_node->id_,this->id_,(int)mr.all_matches.size());
  } 
  else if (!getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches, min_matches) ){ // mr.all_matches.size()/3
      ROS_INFO("Found no valid trafo, but had initially %d feature matches",(int) mr.all_matches.size());
  } else  {
	  ++initial_node_matches_; //trafo is accepted
	  mr.final_trafo = mr.ransac_trafo;
      
#ifdef USE_ICP_CODE
      getRelativeTransformationTo_ICP_code(older_node,mr.icp_trafo, &mr.ransac_trafo);
#endif  
      
#ifdef USE_ICP_BIN
      // improve transformation by using the generalized ICP
      // std::clock_t starttime_gicp1 = std::clock();
      bool converged = getRelativeTransformationTo_ICP_bin(older_node,mr.icp_trafo, &mr.ransac_trafo);
      //ROS_INFO_STREAM("Paper: ICP1: " << ((std::clock()-starttime_gicp1*1.0) / (double)CLOCKS_PER_SEC));


      ROS_INFO("icp: inliers: %i", mr.inlier_matches.size());
      
      if (!converged) { 
        ROS_INFO("ICP did not converge. No Edge added");
        return mr;
      }

      mr.final_trafo = mr.ransac_trafo * mr.icp_trafo;

      MatchingResult mr_icp;
      std::vector<double> errors;
      double error;
      std::vector<cv::DMatch> inliers;
      // check if icp improves alignment:
      computeInliersAndError(mr.inlier_matches, mr.final_trafo,
          this->feature_locations_3d_, older_node->feature_locations_3d_,
          inliers, error, errors, 0.04*0.04); 

      for (uint i=0; i<errors.size(); i++)
      {
        cout << "error: " << round(errors[i]*10000)/100 << endl;
      }

      cout << "error was: " << mr.rmse << " and is now: " << error << endl;

      double roll, pitch, yaw, dist;
      mat2components(mr.ransac_trafo, roll, pitch, yaw, dist);
      cout << "ransac: " << roll << " "<< pitch << " "<< yaw << "   "<< dist << endl;


      mat2components(mr.icp_trafo, roll, pitch, yaw, dist);
      cout << "icp: " << roll << " "<< pitch << " "<< yaw << "   "<< dist << endl;

      mat2components(mr.final_trafo, roll, pitch, yaw, dist);
      cout << "final: " << roll << " "<< pitch << " "<< yaw << "   "<< dist << endl;


      cout << "ransac: " << endl << mr.ransac_trafo << endl;
      cout << "icp: " << endl << mr.icp_trafo << endl;
      cout << "total: " << endl << mr.final_trafo << endl;


      if (error > (mr.rmse+0.02))
      {
        cout << "#### icp-error is too large, ignoring the connection" << endl;
        return mr;
      }
#endif
       

#ifndef USE_ICP_BIN
#ifndef USE_ICP_CODE      
      mr.final_trafo = mr.ransac_trafo;    
#endif
#endif

      mr.edge.id1 = older_node->id_;//and we have a valid transformation
      mr.edge.id2 = this->id_; //since there are enough matching features,
      mr.edge.mean = eigen2G2O(mr.final_trafo.cast<double>());//we insert an edge between the frames

      /*this is a weight, that discounts the information coming from later nodes w.r.t earlier nodes.
      int lowest_id = mr.edge.id1 < mr.edge.id2 ? mr.edge.id1 : mr.edge.id2;
      double timeline_weight = lowest_id > 0 ? (double)lowest_id : 1.0;
      */

      double w = (double)mr.inlier_matches.size();///(double)mr.all_matches.size();
      mr.edge.informationMatrix =   Eigen::Matrix<double,6,6>::Identity()*(w*w); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)
  }
  // Paper
  return mr;
}

void Node::performJointOptimization(const Node* oldNode, MatchingResult& mr)
{
    // RGBD ICP
    ROS_INFO_STREAM("Performing RGBDICP with source node(" << this->id_ << ") and target node (" << oldNode->id_ << ")");

    pcl::IterativeClosestPoint<PointNormal, PointNormal> icp;
    // set source and target clouds from indices of pointclouds
	pcl::ExtractIndices<PointNormal> handlefilter;
	pcl::PointIndices::Ptr sourceHandleIndices (new pcl::PointIndices);
	pointcloud_type::Ptr cloudHandlesSource (new pointcloud_type);
	sourceHandleIndices->indices = this->handleIndices;
	handlefilter.setIndices(sourceHandleIndices);
	handlefilter.setInputCloud(this->pc_col);
	handlefilter.filter(*cloudHandlesSource);
	icp.setInputCloud(cloudHandlesSource);

	pcl::PointIndices::Ptr targetHandleIndices (new pcl::PointIndices);
	pointcloud_type::Ptr cloudHandlesTarget (new pointcloud_type);
	targetHandleIndices->indices = oldNode->handleIndices;
	handlefilter.setIndices(targetHandleIndices);
	handlefilter.setInputCloud(oldNode->pc_col);
	handlefilter.filter(*cloudHandlesTarget);
	icp.setInputTarget(cloudHandlesTarget);

	PointCloudNormal Final;
	icp.align(Final, mr.icp_trafo);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	ROS_INFO("Initialize transformation estimation object....");
	boost::shared_ptr< TransformationEstimationJointOptimize<PointNormal, PointNormal > >
		transformationEstimation_(new TransformationEstimationJointOptimize<PointNormal, PointNormal>());

	float denseCloudWeight = 1.0;
	float visualFeatureWeight = 0.5;
	float handleFeatureWeight = 0.25;
	transformationEstimation_->setWeights(denseCloudWeight, visualFeatureWeight, handleFeatureWeight);

	std::vector<int> sourceSIFTIndices, targetSIFTIndices;
	getFeatureIndices(oldNode,mr,sourceSIFTIndices,targetSIFTIndices);
	transformationEstimation_->setCorrespondecesDFP(sourceSIFTIndices, targetSIFTIndices);

	// custom icp
	ROS_INFO("Initialize icp object....");
	pcl::IterativeClosestPointJointOptimize<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icpJointOptimize; //JointOptimize
	icpJointOptimize.setMaximumIterations (20);
	icpJointOptimize.setTransformationEpsilon (0);
	icpJointOptimize.setMaxCorrespondenceDistance(0.1);
	icpJointOptimize.setRANSACOutlierRejectionThreshold(0.03);
	icpJointOptimize.setEuclideanFitnessEpsilon (0);
	icpJointOptimize.setTransformationEstimation (transformationEstimation_);
	icpJointOptimize.setHandleSourceIndices(sourceHandleIndices->indices);
	icpJointOptimize.setHandleTargetIndices(targetHandleIndices->indices);
	icpJointOptimize.setInputCloud(this->pc_col);
	icpJointOptimize.setInputTarget(oldNode->pc_col);

	ROS_INFO("Running ICP....");
	PointCloudNormal::Ptr cloud_transformed( new PointCloudNormal);
	icpJointOptimize.align ( *cloud_transformed, icp.getFinalTransformation()); //init_tr );
	std::cout << "[SIIMCloudMatch::runICPMatch] Has converged? = " << icpJointOptimize.hasConverged() << std::endl <<
				"	fitness score (SSD): " << icpJointOptimize.getFitnessScore (1000) << std::endl
				<<	icpJointOptimize.getFinalTransformation () << "\n";

	mr.final_trafo = icpJointOptimize.getFinalTransformation();
}

void Node::clearFeatureInformation(){
  //clear only points, by swapping data with empty vector (so mem really gets freed)
	std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > f_l_3d;  
  f_l_3d.swap(feature_locations_3d_);
	std::vector<cv::KeyPoint> f_l_2d; 
  f_l_2d.swap(feature_locations_2d_);
  feature_descriptors_.release();
}
void Node::addPointCloud(pointcloud_type::Ptr new_pc){
  pc_col = new_pc;
}
void Node::clearPointCloud(){
  //clear only points, by swapping data with empty vector (so mem really gets freed)
  pc_col->width = 0;
  pc_col->height = 0;
  pointcloud_type pc_empty;
  pc_empty.points.swap(pc_col->points);
}

// writes the point cloud of a node to file and clears it from memory
void Node::cachePointCloudToFile()
{
	 pcl::PCDWriter writer;
	 std::stringstream filename;
	 filename << "node_" << id_ << ".pcd";
	 writer.write (filename.str(), *pc_col, true);
}

// calcuate the normals of the node's pointcloud
void Node::calculateNormals()
{
	pointcloud_type::Ptr pointCloudOut (new pointcloud_type);
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<point_type, point_type> ne;
	//pcl::NormalEstimationOMP<point_type, point_type> ne;
	//ne.setNumberOfThreads (4);
	ne.setInputCloud (pc_col);
	pcl::search::KdTree<point_type>::Ptr tree (new pcl::search::KdTree<point_type> ());
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.03);
	ne.compute (*pc_col);
}

// calcuate the normals of the node's pointcloud
void Node::getFeatureIndices(const Node* previousNode, MatchingResult& mr,
		std::vector<int>& sourceIndices, std::vector<int>& targetIndices)
{
	pcl::KdTreeFLANN<point_type> kdtreeNN;
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquardDistance(1);
	kdtreeNN.setInputCloud(pc_col);
	for(size_t i = 0; i < mr.inlier_matches.size(); i++)
	{
		point_type searchPoint;
		searchPoint.x = feature_locations_3d_[mr.inlier_matches[i].queryIdx].x();
		searchPoint.y = feature_locations_3d_[mr.inlier_matches[i].queryIdx].y();
		searchPoint.z = feature_locations_3d_[mr.inlier_matches[i].queryIdx].z();
		kdtreeNN.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquardDistance);
		sourceIndices.push_back(pointIdxNKNSearch.front());

		searchPoint.x = previousNode->feature_locations_3d_[mr.inlier_matches[i].trainIdx].x();
		searchPoint.y = previousNode->feature_locations_3d_[mr.inlier_matches[i].trainIdx].y();
		searchPoint.z = previousNode->feature_locations_3d_[mr.inlier_matches[i].trainIdx].z();
		kdtreeNN.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquardDistance);
		targetIndices.push_back(pointIdxNKNSearch.front());
	}

}

void Node::extractHandlesIndices()
{
	HandleExtractor handleExtractor_;
	handleExtractor_.extractHandles(pc_col, handleIndices);
}

void Node::reloadPointCloudFromDisk()
{
	std::stringstream nodeName;
	nodeName << "node_" << id_ << ".pcd";
	ROS_INFO_STREAM("Retrieving node " << nodeName.str() << " from cache and saving to map");
	pcl::PCDReader nodeFetcher;
	nodeFetcher.read(nodeName.str(), *pc_col);
}
