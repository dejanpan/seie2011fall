/*
 * cluster_manager.h
 *
 *  Created on: Jul 26, 2011
 *      Author: engelhar
 */

#ifndef CLUSTER_MANAGER_H_
#define CLUSTER_MANAGER_H_


#endif /* CLUSTER_MANAGER_H_ */

#include <kidnapped_robot/place_database.h>

#include <frame_common/stereo.h>
#include <frame_common/frame.h>

//#include <g2o/types/slam3d/vertex_depthcam.h>
//#include <g2o/types/slam3d/vertex_trackxyz.h>
//#include <g2o/types/slam3d/edge_project_disparity.h>
#include <posest/pe3d.h>

#include "node_definitions.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>







Eigen::Matrix4d combine(Eigen::Matrix3d& rot, Eigen::Vector3d& trans);


class Cluster_manager {
public:


  void tree_check_incremental(uint ignored_last,int tree_hits);
  void tree_check_batch(uint ignored_last,int tree_hits);


  void storeNode(Node& node);
  bool appendClusterToFile(cluster& c, ofstream& off);
  int writePosesToFile(string filename, bool ignore_unoptimized = true);

  bool appendNodeToFile(Node* node);


  vt::Database* vt_db;

  ros::Publisher pub_new;
  ros::Publisher pub_old;
  ros::Publisher pub_new_moved;
  ros::Publisher pub_marker_array;
  void sendClouds(ros::Publisher pub, Pose_Type pt, string frame);

  LM_map landmarks;
  uint next_land_mark_id;

  void updateLandmarks(Node& new_, Node& other, vector<cv::DMatch>& matches);

  pe::PoseEstimator3d* pose_estimator_;
  kidnapped_robot::PlaceDatabase* place_db_;
  node_map nodes;

  // ids of all clusters which contain a node that matched with the new_node
  set<uint> matched_clusters;

  int delete_unimportant_clusters();


  bool isClusterConnected(cluster& c);

  void ConsolidateCluster();

  int mergeClusters(cluster& c1, cluster& c2); // search for new node-connections

  void combineCluster(set<uint>& cluster_ids); // simply copies nodes into first cluster


  cluster_map clusters;
  uint next_cluster_id;

  // Node dangling;// last non-matching frame
  // bool dangling_valid;

  Unique_Ring_buffer<int>* last_matches;

  // true if node was added
  // bool addNode(Node& new_node);
  // void addNode_simple(Node& new_node);

  /*
   * includes new node, merges cluster
   * returns cluster_ids of merged clusters
   */
  set<uint> addNode_tree_n_last(Node& new_node); //TODO: better name


  void processNode(Node& new_node);

  // add node to nodes and last_added_nodes (and possibly a node into the tree)
  void appendNode(Node& node);



  bool check_pair(Node * new_node, Node* other, Edge_type type,int& inliers,  float* d = NULL, float* phi = NULL);
  bool EvaluateMatch(Node* n1, Node* n2,Node_match& match, error_set& es);

  set<uint> tested; // neighbours or tree recommendations
  set<uint> hits;       // matches
  set<uint> tree_hits;  // matched tree recommendations


  // find N neighbours with the tree
  uint n_neighbours;
  // the last N nodes are added to the node_map, but not yet to the tree
  // every new node is compared the the last added nodes and those found in the tree
  // s.t. there are at most (C_N_neighbours+C_max_stasis_cnt) node-node comparisons
  uint max_stasis_cnt;

  // min number of inliers to accept as match
  int min_inlier_cnt;

  // id in docDatabase 2 to id in nodes
  map<uint,uint> treeId2NodeId;

  // min dist to keyframe
  float min_dist_cm;
  float min_rot_deg;

  int search_depth;

  // performs posest, returns true if nodes match and dist/rot is not too small
  bool isNewKeyframe(Node* new_);

  // new keyframe:
  // a) no match
  // b) trafo larger than C_min-params
  int keyframe;
  bool last_node_was_keyframe;

  int last_nodes_cnt;


  // matches with nodes that are far away
  set<uint> far_hits;


  int64 ticks_for_ransac;
  uint node_comparisons;
  void printTiming();

  void writeImagesAtClusterBorders();


  // added nodes are not immediately added to the vocab_tree, but compared directly
  vector<uint> last_added_nodes;

  ofstream* match_stream;
  ofstream* point_error_stream;
  ofstream* all_comparisons;


  Cluster_manager(uint queue_size = 0, int tree_neighbour_cnt = 15 ,int  min_inliers = 30, int last_matches_ring_size = 10);

  /*
   * find all nodes which are connected to
   */
  void depthSearch(uint id, uint depth, set<uint>* found, set<uint>* black_list);

  int addMatchedWordsToTree(Node* node);
  static void getTransformed(const point_cloud& in, const Eigen::Matrix4d&  trafo, point_cloud& out);
  static void mat2RPY(const Eigen::Matrix3d& t, double& roll, double& pitch, double& yaw);

};




namespace cluster_optimization {

void populateOptimizer(Cluster_manager& cm, cluster* cl, g2o::SparseOptimizer* optimizer, uint& vertexCounter, set<uint>& optimized_nodes, bool fix_first_in_cluster = true);


void optimize_cluster(Cluster_manager& cm, cluster* cl,int iterations = 15);
void optimize_with_landmarks(Cluster_manager& cm, int iterations = 15);

void optimize_with_tracker(Cluster_manager& cm, int iterations = 30);

void updatePoses(Cluster_manager& cm, g2o::SparseOptimizer* optimizer, set<uint>& optimized_nodes);

void setPoseWithConstantvelocity(Cluster_manager& cm, uint node_id);


int addTrackerEdges(Cluster_manager& cm, g2o::SparseOptimizer* optimizer, set<uint>& ids);
int addConsecutiveEdges(Cluster_manager& cm, g2o::SparseOptimizer* optimizer, set<uint>& ids_set);
int addEdgesForSoloNodes(Cluster_manager& cm, g2o::SparseOptimizer* optimizer, set<uint>& ids_set);

void initOptimizer(g2o::SparseOptimizer* optimizer, frame_common::CamParams cam);

}

