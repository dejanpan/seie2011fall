/*
 * node_collection.h
 *
 *  Created on: Jul 20, 2011
 *      Author: engelhar
 */

#ifndef NODE_COLLECTION_H_
#define NODE_COLLECTION_H_

#include <kidnapped_robot/place_database.h>

#include <frame_common/stereo.h>
#include <frame_common/frame.h>
#include "ground_truth.h"


//#include <g2o/types/slam3d/vertex_depthcam.h>
//#include <g2o/types/slam3d/vertex_trackxyz.h>
//#include <g2o/types/slam3d/edge_project_disparity.h>
#include <g2o/types/slam3d/vertex_se3_quat.h>
#include <g2o/types/slam3d/camera_parameters.h>
#include <g2o/types/slam3d/vertex_trackxyz.h>
#include <g2o/types/slam3d/edge_project_disparity.h>
#include <g2o/types/slam3d/edge_se3_quat.h>

#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>

typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
typedef set<uint>::iterator u_it;

#include <image_geometry/stereo_camera_model.h>

struct node_pair {
  uint node_id1;
  uint node_id2;


  node_pair(uint n1, uint n2, double d){
    node_id1 = n1;
    node_id2 = n2;
    distance = d;
  }

  // the lower the better
  double distance;

};

struct node_pair_comp {
  bool operator() (node_pair n1,node_pair n2) { return (n1.distance<n2.distance);}
};


// small hacked ringbuffer with unique elements

template <class T>
struct Unique_Ring_buffer {

  unsigned int size_;

  Unique_Ring_buffer(unsigned int size) {
    size_ = size;
    reset();
  }

  bool add(T obj){

    // check if object in list:
    for (unsigned int i=0; i<v.size(); ++i)
      if (v[i] == obj) return false;

    v.push_back(obj);
    if (v.size() > size_)
      v.erase(v.begin());

    assert(v.size() <= size_);
    return true;
  }



  vector<T> v;

  void reset() {v.clear();}

};



struct error_set {
  float mean_;
  float max_;
  int N;
  Eigen::Vector3d mean_abs_dist;
};



enum Pose_Type {
  PT_ground_truth,
  PT_inital_estimate,
  PT_optimized
};


enum Edge_type {
  ET_Keyframe_check,
  ET_Direct_neighbour,
  ET_Tree_Proposal,
  ET_Ring,
  ET_Last_match,
  ET_Far_match,
  ET_CLUSTER_MERGE
};


// result of frame comparison
struct Node_match {

  uint other_id; // id of other Node
  float tree_score; // score at the tree-test
  std::vector<cv::DMatch> inlier;


  Edge_type type;

  // add edge

  // relative position as computed by RANSAC
  Eigen::Vector3d trans;
  Eigen::Matrix3d rot;
  float dist; // norm of trans

  uint getInlierCount() {return inlier.size();}
  float getDist() {return trans.norm(); }

  // used to find best matching partner to initialize position
  bool operator<(Node_match& other) {
    return (this->getInlierCount() < other.getInlierCount()); };

  void invert(uint new_id){
    other_id = new_id;

    for (uint i=0; i<inlier.size(); ++i)
      swap(inlier.at(i).queryIdx,inlier.at(i).trainIdx);

    // y = R*x+t <=> x = R^-1 + (- R^{-1} * t)
    Eigen::Matrix3d inv = rot.inverse();
    rot = inv;
    trans = -inv*trans;
  }


};

struct LM_Observation {

  uint node_id;
  uint pt_id;

  // g2o::EdgeProjectDisparity * projectionEdge;

  LM_Observation(uint node, uint pt){
    node_id = node;
    pt_id = pt;
    // projectionEdge = NULL;
  }

  // ~Observation() { if (projectionEdge != NULL) delete projectionEdge; }

};

struct lt_obs{
  bool operator()(const LM_Observation& A, const LM_Observation& B) const
  {
    if (A.node_id == B.node_id)
      return A.pt_id < B.pt_id;
    return A.node_id < B.node_id;
  }
};

struct lt_cv_cmatch_query{
  bool operator()(const cv::DMatch& A, const cv::DMatch& B) const
  {
    return A.queryIdx < B.queryIdx;
  }
};

struct lt_cv_cmatch_train{
  bool operator()(const cv::DMatch& A, const cv::DMatch& B) const
  {
    return A.trainIdx < B.trainIdx;
  }
};


// the landmarks remembers all of its observations
struct Landmark {
  vector<LM_Observation> observations;
  g2o::VertexTrackXYZ * trackVertex;
  // Eigen::Vector4d pose;

  uint id;
  uint getObservationCnt(){ return observations.size(); }

  Landmark(){ trackVertex = NULL; }
  ~Landmark(){
    //ROS_INFO("destructor start");
    // if (trackVertex!=NULL) delete trackVertex;
    //ROS_INFO("destructor end");
  }
};


typedef map<uint, Landmark> LM_map;
typedef map<uint, Landmark>::iterator LM_iter;







struct Node {

  uint cluster_id;
  uint node_id;
  uint odo_id;
  frame_common::Frame frame;

  cv::Mat depth; // used to store depth to create pointcloud if needed for icp
  bool has_dense_pointcloud;

  image_geometry::PinholeCameraModel cam_model;
  bool is_keyframe;
  bool was_optimized;

  g2o::VertexSE3* camVertex;
  bool is_fixed; // true iff camVertex is fixed


  cv::Point2f gt_pos_xy;

  // <id, inlier>: this node was compared to the given other node and so many inlier were found
  map<uint,uint> comparisons;


  //void getTransformed(const point_cloud& in, const Eigen::Matrix4d&  trafo, point_cloud& out);
  // destroys density of cloud


  vector<int> tree_proposals;


  Eigen::Matrix4d getPose(){
    return (was_optimized)?opt_pose:pose;
  }

  Eigen::Matrix4d pose;
  Eigen::Matrix4d opt_pose;
  Eigen::Matrix4d opt_init_pose;

  Eigen::Matrix4d ground_truth_pose;
  bool ground_truth_valid;

  uint isConnected() {return matches.size() > 0;}

  vt::Document words;
  vector<Node_match> matches;

  map<uint,uint> pt_to_landmark; // <f,l> feature f has seen landmark l

  ros::Time stamp;

  Node(){
    camVertex = NULL;
    ground_truth_pose = Eigen::Matrix4d::Identity();
    ground_truth_valid = false;
    is_keyframe = false;
    pose = Eigen::Matrix4d::Identity();
    opt_pose = Eigen::Matrix4d::Identity();
    is_fixed = false;
    was_optimized = false;
    has_dense_pointcloud = false;
  }
  // Node() { depthCamVertex = NULL; pose = Eigen::Matrix4d::Identity(); }
  // ~Node() { if (depthCamVertex != NULL) delete depthCamVertex; }


};

// contains node_ids of connected nodes
struct cluster {
  uint cluster_id;
  vector<uint> node_ids;

  bool changed;

  cluster(){changed = true;}

  bool operator<(cluster other){ return cluster_id < other.cluster_id; }




};


typedef map<uint,cluster> cluster_map;
typedef cluster_map::iterator clm_it;

// collection of nodes
typedef map<uint,Node> node_map;
typedef node_map::iterator node_it;


#endif /* NODE_COLLECTION_H_ */
