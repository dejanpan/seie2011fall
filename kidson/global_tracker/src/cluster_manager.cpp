/*
 * cluster_manager.cpp
 *
 *  Created on: Jul 26, 2011
 *      Author: engelhar
 */

#include "cluster_manager.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "visualization.h"

// #include "optimizer.h"


#define USE_SURF_TREE



void Cluster_manager::tree_check_batch(uint ignored_last,int tree_hits){
  ofstream tree_neighours;
  tree_neighours.open("/home/engelhar/Desktop/evaluation/tree_batch.txt");

//  uint ignored_last = 10; // dont use
//  int tree_hits = 20; // number of tree hits to store

  // vt::Database db;

  // add all nodes to the tree
//  for (uint i=0; i<nodes.size(); ++i){
//    ROS_WARN("add node %i to tree", i);
//    Node* current = &nodes[i];
//    int doc_id = place_db_->getDocDatabase()->insert(current->words);
//    treeId2NodeId[doc_id] = current->node_id;
//  }

  // and find possible neighbours
  for (uint i=0; i<nodes.size(); ++i){
    Node* current = &nodes[i];
    vt::Matches matches;
    matches.clear();

#ifdef USE_SURF_TREE
    vt_db->find(current->words, tree_hits, matches);
#else
    place_db_->getDocDatabase()->find(current->words, tree_hits, matches);
#endif

    for (uint m=0; m < matches.size(); ++m){

      map<uint,uint>::iterator it = treeId2NodeId.find(matches.at(m).id);
      if (it == treeId2NodeId.end()) {
        ROS_INFO("wrong id: %i", matches.at(m).id);
        continue;
      }



      int diff = int(i)-int(it->second);
      if (diff<0) diff = -diff;
      if ( diff >= ignored_last ){
        tree_neighours << i << "  " << it->second << "  " << m << endl;
        // cout << "add " << i << "  " << it->second << endl;
      }
    }

  }

}

void Cluster_manager::tree_check_incremental(uint ignored_last,int tree_hits ){

  ofstream tree_neighours;
  tree_neighours.open("/home/engelhar/Desktop/evaluation/tree_incremental.txt");

//  uint ignored_last = 10; // dont use
//  int tree_hits = 20; // number of tree hits to store


  treeId2NodeId.clear();

  //vt::Database db;

  for (uint i=0; i<nodes.size(); ++i){
    ROS_WARN("neighbours for node %i", i);

//    for (map<uint,uint>::iterator it = treeId2NodeId.begin(); it != treeId2NodeId.end(); ++it){
//      ROS_INFO("state: %i %i", it->first, it->second);
//    }

    Node* current = &nodes[i];
    vt::Matches matches;
    matches.clear();

#ifdef USE_SURF_TREE
    vt_db->find(current->words, tree_hits, matches);
#else
    place_db_->getDocDatabase()->find(current->words, tree_hits, matches);
#endif
    //    ROS_WARN("matches.size: %i", int(matches.size()));


    for (uint m=0; m < matches.size(); ++m){

      map<uint,uint>::iterator it = treeId2NodeId.find(matches.at(m).id);

      if (it == treeId2NodeId.end()) {
        ROS_INFO("wrong id: %i", matches.at(m).id);
        continue;
      }

      ROS_INFO("match_id %i, node: %i", it->first, it->second);
      {
        //        ROS_INFO("%i is in the map", matches.at(m).id);
        tree_neighours << i << "  " << it->second << "  " << m << endl;
      }
    }


    // insert nodes to tree
    if (i >= ignored_last){
      Node* add = &nodes[i-ignored_last];
#ifdef USE_SURF_TREE
      int doc_id = vt_db->insert(add->words);
#else
    int doc_id = place_db_->getDocDatabase()->insert(add->words);
#endif

      treeId2NodeId[doc_id] = add->node_id;
      ROS_ERROR("adding node %i to tree with docId %i", int(add->node_id), doc_id);
    }

  }


  //  vt::Matches matches;
  //  place_db_->getDocDatabase()->find(new_node.words, n_neighbours, matches );
  //  ROS_INFO("Tree Recommendations: ");
  //  for (uint i=0; i < matches.size() ; ++i){
  //    int id = treeId2NodeId[matches.at(i).id];
  //    new_node.tree_proposals.push_back(id);
  //
  //    if (check_pair(&new_node, &(nodes[id]), ET_Tree_Proposal, inlier_cnt)) { tree_hits.insert(id); }
  //    //        ROS_INFO("tree: %i, inls: %i", id, inlier_cnt);
  //  }
  //
  //  int doc_id = place_db_->getDocDatabase()->insert(n->words);
  //
  //  if (doc_id > 0){
  //    // ROS_INFO("Added node %i with %i points to tree_id %i", n->node_id, (int)  n->words.size(), doc_id);
  //
  //    treeId2NodeId[doc_id] = n->node_id;
  //  }


}




void Cluster_manager::printTiming(){

  double secs = ticks_for_ransac/cv::getTickFrequency();
  double secs_per_check = secs*1.0/node_comparisons;

  ROS_ERROR("TIMING for comparisons: %f s for %i comps (%f s each)", secs, node_comparisons, secs_per_check);

}



Eigen::Matrix4d combine(Eigen::Matrix3d& rot, Eigen::Vector3d& trans){
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

  for (int y=0; y<3; y++){
    pose(y,3) = trans(y);
    for (int x=0; x<3; x++)
      pose(y,x) = rot(y,x);
  }

  return pose;
}







float node_projection_dist(Node* n1, Node* n2, int n_tries){

  Eigen::Matrix4d pose1 = n1->getPose();
  Eigen::Matrix4d pose2 = n2->getPose();

  int hit_cnt = 0;


  // Those points are projected into other image to estimate the overlapping field of view
  //  vector<cv::Point2i> sample_points;
  //  vector<float> depths;
  //  for (float d = 0.5; d<=3; d+=0.5) depths.push_back(d);
  //
  //  // 3x4-array without center points in a 640x480 img
  //  sample_points.push_back(cv::Point2i(64      ,80));
  //  sample_points.push_back(cv::Point2i(64+1*128,80));
  //  sample_points.push_back(cv::Point2i(64+2*128,80));
  //  sample_points.push_back(cv::Point2i(64+3*128,80));
  //
  //  sample_points.push_back(cv::Point2i(64      ,80+160));
  //  sample_points.push_back(cv::Point2i(64+3*128,80+160));
  //
  //  sample_points.push_back(cv::Point2i(64      ,80+2*160));
  //  sample_points.push_back(cv::Point2i(64+1*128,80+2*160));
  //  sample_points.push_back(cv::Point2i(64+2*128,80+2*160));
  //  sample_points.push_back(cv::Point2i(64+3*128,80+2*160));
  //
  //
  //  vector<Eigen::Vector4d> points(depths.size()*sample_points.size());
  //  Eigen::Vector4d p_4d;
  //  for (uint i=0; i<depths.size(); ++i)
  //    for (uint j=0; j<sample_points.size(); ++j){
  //      cv::Point3d p = n1->cam_model.projectPixelTo3dRay(sample_points[j])*depths[i];
  //      p_4d.array()[0] = p.x;
  //      p_4d.array()[1] = p.y;
  //      p_4d.array()[2] = p.z;
  //      p_4d.array()[3] = 1;
  //      points.push_back(p_4d);
  //    }


  //  int N = points.size();

  int N = n1->frame.pts.size();

  // project some points into other image:
  for (int j=0; j < N; ++j){

    Eigen::Vector4d pt_w = pose1*n1->frame.pts[j];
    Eigen::Vector4d pt_2 = pose2.inverse()*pt_w;


    // point is behind camera
    if (pt_2.array()[2] < 0) continue;

    cv::Point2f cv_2p =  n2->cam_model.project3dToPixel(cv::Point3d(
        pt_2.array()[0],
        pt_2.array()[1],
        pt_2.array()[2] ));


    //        ROS_INFO("%i: pt %i is projected to %f %f", j, pt_ndx, cv_2p.x, cv_2p.y );

    if (0 <= cv_2p.x && cv_2p.x < 640 &&
        0 <= cv_2p.y && cv_2p.y < 480){

      // ROS_INFO("inlier");

      hit_cnt++;

    }

  }

  //  ROS_INFO("%i of %i pts projected into image", hit_cnt, N);

  return hit_cnt*1.0/N;

}


float node_projection_dist_sym(Node* n1, Node* n2, int n_tries){

  float d1 = node_projection_dist(n1, n2, n_tries);
  float d2 = node_projection_dist(n2, n1, n_tries);

  return (d1+d2)/2;

}

//
//
//// find possible close nodes and compare with them
//void Cluster_manager::FindPoseNeighbours(Node* node){
//
//  int min_id_dist = 15; // don't compare with nodes which are too close
//
//
//  for (node_map::iterator it = nodes.begin(); it != nodes.end(); ++it)
//  {
//
//    Node* other = &nodes[*it];
//
//
//
//
//    Eigen::Matrix4d pose1 = n1->getPose();
//    Eigen::Matrix4d pose2 = n2->getPose();
//
//    Eigen::Matrix4d rel = pose1.inverse()*pose2;
//
//    double r,p,y;
//    mat2RPY(rel.block<3,3>(0,0),r,p,y);
//
//    double alpha = max(max(abs(r),abs(p)),abs(y))/M_PI*180.0;
//
//    Eigen::Vector4d tr = rel.col(3); tr.array()[3] = 0;
//    double d = tr.norm()*100;
//
//    //      cout << "pose 1" << endl;
//    //      cout << pose1 << endl;
//    //      cout << "pose 2" << endl;
//    //      cout << pose2 << endl;
//
//
//
//    //      float dist = d+alpha;
//    //      if (dist > 100)
//    //        continue;
//
//
//    //      ROS_INFO("d: %f, a: %f, dist: %f ", d, alpha, dist);
//
//    if (d>50) continue;
//    if (alpha > 30) continue;
//
//    // dist: in [0,1], 1 being best
//    //      float dist = node_projection_dist_sym(n1, n2, 0);
//    //
//    //      // at least 10% hits
//    //      if (dist < 0.3)
//    //        continue;
//
//    //      ROS_INFO("pair: %i %i  %f", *it1, *it2, dist);
//    //      ROS_INFO("alpha: %f, dist: %f", alpha, d);
//
//    // invert score, s.t. ascending sort still works
//    pairs.push_back(node_pair(*it1, *it2, 3*d+alpha));
//
//  }
//
//  ROS_INFO("cluster merge: found %i possible pairs (from %i)", int(pairs.size()), int(c1.node_ids.size()*c2.node_ids.size()));
//
//  node_pair_comp comp;
//  sort(pairs.begin(), pairs.end(), comp);
//
//  // check best pairs
//  //  ROS_INFO("tested pairs (dist should be ascending)");
//
//  uint hit_cnt = 0;
//  int inliers;
//  int max_cnt = min(max_check_cnt, uint(pairs.size()));
//  for (int i=0; i <max_cnt; ++i){
//
//    node_pair np = pairs[i];
//    ROS_INFO("pair: %i %i dist: %f", np.node_id1, np.node_id2, np.distance);
//
//    bool hit = check_pair(&nodes[np.node_id1], &nodes[np.node_id2], ET_CLUSTER_MERGE, inliers);
//
//    if (hit) hit_cnt++;
//
//  }
//
//
//
//}



void Cluster_manager::writeImagesAtClusterBorders(){

  //  int N = 3; // N images before and after clusterbreak

  for (cluster_map::iterator it = clusters.begin(); it != clusters.end(); ++it){

    sort(it->second.node_ids.begin(),it->second.node_ids.end());





  }


}





/*
 * try to combine the clusters
 */
void Cluster_manager::ConsolidateCluster(){



  vector<uint> cluster_ids;

  // get list of all cluster ids
  for (cluster_map::iterator it = clusters.begin(); it != clusters.end(); ++it){
    if (it->second.node_ids.size() > 5)
      cluster_ids.push_back(it->second.cluster_id);
  }

  if (cluster_ids.size() < 2)
    return;

  typedef pair<uint,uint> u_pair;

  vector<u_pair> pairs;
  for (uint i=0; i<cluster_ids.size()-1; ++i)
    for (uint j=i+1; j<cluster_ids.size(); ++j){
      pairs.push_back(u_pair(cluster_ids[i],cluster_ids[j]));
    }


  ROS_INFO("CONSOLIDATE:");
  for (uint i=0; i<pairs.size(); ++i){
    uint id1 = pairs[i].first;
    uint id2 = pairs[i].second;
    ROS_INFO("Plan: comp cluster %i vs %i", id1, id2);
  }

  // try to merge all pairs of clusters:
  for (uint i=0; i<pairs.size(); ++i){
    uint id1 = pairs[i].first;
    uint id2 = pairs[i].second;

    // node was already merged
    if (clusters.find(id1) == clusters.end()) continue;
    if (clusters.find(id2) == clusters.end()) continue;

    int hits = mergeClusters(clusters[id1],clusters[id2]);

    // no new match, clusters can't be merged
    if (hits == 0) continue;

    set<uint> matched; matched.insert(id1); matched.insert(id2);

    assert(id1<id2);
    combineCluster(matched); // copy second cluster into first one

    clusters.erase(id2);
  }

}





int Cluster_manager::mergeClusters(cluster& c1, cluster& c2){


  ROS_ERROR("Merging clusters %i and %i", c1.cluster_id, c2.cluster_id);

  //  double max_dist = 50; // summe aus dist in cm und winkel in deg
  uint max_check_cnt = 50;

  vector<node_pair> pairs;


  // problem: the super_cluster contains all merged clusters, s.t. a cluster is also compared to itself
  // if we would simply compare all clusters pairwise
  // easy and ugly solution: compare supercluster with all others and remove current cluster from it

  // HACKAGHCAKACHGACHACHKACKHAC, aber egal

  sort(c1.node_ids.begin(), c1.node_ids.end());
  sort(c2.node_ids.begin(), c2.node_ids.end());
  vector<uint> c1_without_c2(c1.node_ids.size());
  vector<uint>::iterator it_end;
  it_end = set_difference (c1.node_ids.begin(), c1.node_ids.end(), c2.node_ids.begin(), c2.node_ids.end(), c1_without_c2.begin());

  //  for (vector<uint>::iterator it1 = c1_without_c2.begin(); it1 != it_end; ++it1)
  //    ROS_INFO("Node in first cluster: %i", *it1);
  //  ROS_ERROR("break");
  //  for (vector<uint>::iterator it1 = c2.node_ids.begin(); it1 != c2.node_ids.end(); ++it1)
  //      ROS_INFO("Node in second cluster: %i", *it1);


  for (vector<uint>::iterator it1 = c1_without_c2.begin(); it1 != it_end; ++it1)
    for (vector<uint>::iterator it2 = c2.node_ids.begin(); it2 != c2.node_ids.end(); ++it2){

      Node* n1 = &nodes[*it1];
      Node* n2 = &nodes[*it2];

      Eigen::Matrix4d pose1 = n1->getPose();
      Eigen::Matrix4d pose2 = n2->getPose();

      Eigen::Matrix4d rel = pose1.inverse()*pose2;

      double r,p,y;
      mat2RPY(rel.block<3,3>(0,0),r,p,y);

      double alpha = max(max(abs(r),abs(p)),abs(y))/M_PI*180.0;

      Eigen::Vector4d tr = rel.col(3); tr.array()[3] = 0;
      double d = tr.norm()*100;

      //      cout << "pose 1" << endl;
      //      cout << pose1 << endl;
      //      cout << "pose 2" << endl;
      //      cout << pose2 << endl;



      //      float dist = d+alpha;
      //      if (dist > 100)
      //        continue;


      //      ROS_INFO("d: %f, a: %f, dist: %f ", d, alpha, dist);

      if (d>50) continue;
      if (alpha > 30) continue;

      // dist: in [0,1], 1 being best
      //      float dist = node_projection_dist_sym(n1, n2, 0);
      //
      //      // at least 10% hits
      //      if (dist < 0.3)
      //        continue;

      //      ROS_INFO("pair: %i %i  %f", *it1, *it2, dist);
      //      ROS_INFO("alpha: %f, dist: %f", alpha, d);

      // invert score, s.t. ascending sort still works
      pairs.push_back(node_pair(*it1, *it2, 3*d+alpha));

    }

  ROS_INFO("cluster merge: found %i possible pairs (from %i)", int(pairs.size()), int(c1.node_ids.size()*c2.node_ids.size()));

  node_pair_comp comp;
  sort(pairs.begin(), pairs.end(), comp);

  // check best pairs
  //  ROS_INFO("tested pairs (dist should be ascending)");

  uint hit_cnt = 0;
  int inliers;
  int max_cnt = min(max_check_cnt, uint(pairs.size()));
  for (int i=0; i <max_cnt; ++i){

    node_pair np = pairs[i];
    ROS_INFO("pair: %i %i dist: %f", np.node_id1, np.node_id2, np.distance);

    bool hit = check_pair(&nodes[np.node_id1], &nodes[np.node_id2], ET_CLUSTER_MERGE, inliers);

    if (hit) hit_cnt++;

  }


  ROS_INFO("clustermerge ended: found %i new connections (%i attempts)", hit_cnt, max_cnt);

  return hit_cnt;


}


Cluster_manager::Cluster_manager(uint queue_size, int tree_neighbour_cnt,int  min_inliers, int last_matches_ring_size){
  min_inlier_cnt = min_inliers;
  n_neighbours = tree_neighbour_cnt;
  max_stasis_cnt = queue_size;
  // dangling_valid = false;
  last_node_was_keyframe = false;

  ticks_for_ransac = 0;
  node_comparisons = 0;


  next_cluster_id = 0;
  next_land_mark_id = 0;
  last_matches = new Unique_Ring_buffer<int>(last_matches_ring_size);
  match_stream = NULL;
  keyframe = -1;
  min_dist_cm = 10;
  min_rot_deg = 5;

}



bool Cluster_manager::isClusterConnected(cluster& c){
  set<uint> expanded;
  stack<uint> stack_;


  expanded.insert(c.node_ids[0]);
  stack_.push(c.node_ids[0]);

  while(stack_.size()>0){

    uint n_id = stack_.top(); stack_.pop();

    Node* n = & nodes[n_id];
    //    ROS_INFO("expanding node %u", n->node_id);
    for (uint i=0; i<n->matches.size(); ++i)
    {
      uint other = n->matches[i].other_id;

      //      ROS_INFO("found node %u", other);
      if (expanded.find(other) != expanded.end())
        continue;

      //      ROS_INFO("pushing %i to stack", other);
      expanded.insert(other);
      stack_.push(other);
    }
  }

  if (expanded.size() != c.node_ids.size())
    ROS_INFO("found %i nodes by depth-search in cluster %i, expected %i", (int) expanded.size(), (int) c.cluster_id, (int) c.node_ids.size());


  return expanded.size() == c.node_ids.size();
}



bool Cluster_manager::appendClusterToFile(cluster& c, ofstream& off){

  assert(off.is_open());

  for (uint i = 0; i<c.node_ids.size(); ++i)
  {

    Node* n = &nodes[c.node_ids[i]];

    //    Eigen::Matrix4d pose = n->getPose();
    Eigen::Matrix4d pose = n->opt_pose;

    Eigen::Quaterniond quat(pose.topLeftCorner<3,3>());
    off << n->stamp << "  " << pose(0,3) << "  " << pose(1,3) << "  " << pose(2,3) << "    ";
    off <<  quat.x() << "   " << quat.y() << "  " << quat.z() << "  " << quat.w() << endl;

  }
  return true;
}



int Cluster_manager::writePosesToFile(string filename, bool ignore_unoptimized){


  ofstream off;
  off.open(filename.c_str());

  if (!off.is_open())
    return -1;


  int opt_cnt = 0;
  for (node_it it = nodes.begin(); it != nodes.end(); ++it){

    Node* n = &it->second;

    if (ignore_unoptimized && !n->was_optimized)
      continue;

    opt_cnt++;

    Eigen::Matrix4d pose = n->getPose();
    //    for (int i=0; i<4; i++)
    //     for (int j=0; j<4; j++){
    //      off << pose(i,j) << ",  ";
    //    }

    Eigen::Quaterniond quat(pose.topLeftCorner<3,3>());
    off << n->stamp << "  " << pose(0,3) << "  " << pose(1,3) << "  " << pose(2,3) << "    ";
    off <<  quat.x() << "   " << quat.y() << "  " << quat.z() << "  " << quat.w() << endl;

  }

  off.close();

  return opt_cnt;
}

void Cluster_manager::depthSearch(uint id, uint depth, set<uint>* found, set<uint>* black_list){
  if (depth == 0) return;

  //  ROS_INFO("start search for node %i", (int) id);

  Node* current = &nodes[id];
  for (uint i=0; i<current->matches.size(); ++i){
    int new_id = current->matches[i].other_id;
    //  ROS_INFO("%i matches with %i", current->node_id, new_id);
    if (black_list->find(new_id) == black_list->end()){
      found->insert(new_id);
      black_list->insert(new_id);
      depthSearch(new_id, depth-1, found, black_list);
    }
  }
}

///Get euler angles from affine matrix
void Cluster_manager::mat2RPY(const Eigen::Matrix3d& t, double& roll, double& pitch, double& yaw) {
  roll = atan2(t(2,1),t(2,2));
  pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
  yaw = atan2(t(1,0),t(0,0));
}

//void getTrueDistBetween(Eigen::Matrix4d* p1, Eigen::Matrix4d* p2,float& dist, float& max_angle){
//
//  Eigen::Matrix4d rel =  p2*p1.inverse();
//
//  double r,p,y;
//  mat2RPY(rel.block<3,3>(0,0),r,p,y);
//
//  max_angle = max(max(abs(r),abs(p)),abs(y))/M_PI*180.0;
//
//  Eigen::Vector4d tr = rel.col(3); tr.array()[3] = 0;
//  dist = tr.norm()*100;
//
//}

bool Cluster_manager::EvaluateMatch(Node* n1, Node* n2,Node_match& match, error_set& es){

  if (!n1->ground_truth_valid || !n2->ground_truth_valid)
  {
    return false;
  }
  //  if (n1->node_id < n2->node_id + 10)
  //    return false;


  Eigen::Matrix4d gt_1 = n1->ground_truth_pose;
  Eigen::Matrix4d gt_2 = n2->ground_truth_pose;


  ROS_ASSERT(n2->node_id == match.other_id);


  Eigen::Vector4d p1_loc, p1_glob, p1_glob_s;
  Eigen::Vector4d p2_loc, p2_glob, p2_glob_s;
  Eigen::Vector4d error;
  uint match_cnt = match.inlier.size();


  Eigen::Matrix4d rel_guessed = combine(match.rot, match.trans);
  Eigen::Matrix4d rel_correct = gt_2.inverse()*gt_1;

  //  double r,p,y;
  //  mat2RPY(rel_correct.block<3,3>(0,0),r,p,y);

  Eigen::Vector4d tr = rel_correct.col(3); tr.array()[3] = 0;
  // double true_dist = tr.norm()*100;
  //  printf("rpy: %f %f %f \n", r/M_PI*180,p/M_PI*180,y/M_PI*180 );

  // ROS_INFO("dist: %f", true_dist);


  float dists[match_cnt];

  Eigen::Vector3d mean_error(0,0,0);

  for (uint i=0; i<match_cnt; ++i){
    cv::DMatch* m = &match.inlier[i];

    p1_loc = n1->frame.pts[m->queryIdx];
    p2_loc = n2->frame.pts[m->trainIdx];


    p1_glob = gt_1*p1_loc;
    p2_glob = gt_2*p2_loc;



    error = (p1_glob-p2_glob);
    double dist = error.norm()*100;

    for (int j=0; j<3; ++j)
      mean_error.array()[j] += abs(error.array()[j]);

    dists[i] = dist;
    m->distance = dist;

  }


  if ((int) match_cnt >=  min_inlier_cnt)
  {
    //    cv::Mat match_img;
    //    drawColoredMatches(*n1, *n2,match.inlier,match_img);
    //    char buffer[400];
    //    sprintf(buffer, "/export/home/engelhar/imgs/%i_%i__%i.%i_%i.%i.jpg",
    //            int(es.max_), match_cnt,
    //            n1->stamp.sec, n1->stamp.nsec,
    //            n2->stamp.sec, n2->stamp.nsec);
    //    cv::imwrite(buffer, match_img);
    //
    //    cv::namedWindow("matches",1);
    //    cv::imshow("matches", match_img);
    //    cv::waitKey(20);



    for (int j=0; j<3; ++j)
      mean_error.array()[j] /= match_cnt;


    float d_mean = 0;
    float d_max = 0;
    for (uint i=0; i<match_cnt; ++i){
      float d = dists[i];
      d_max = max(d_max,d);
      d_mean += d;
    }

    d_mean /= match_cnt;


    es.N = match_cnt;
    es.max_ = d_max;
    es.mean_ = d_mean;
    es.mean_abs_dist = mean_error;

    // ROS_INFO("%i %i: %i points: max: %f, mean: %f", n1->node_id, n2->node_id,  (int) match_cnt, es.max_, es.mean_);
    // cout << "mean error: " << mean_error << endl;


    char buff[200];
    sprintf(buff, "%i.%i, %i.%i,  %i, %f, %f",
            n1->stamp.sec, n1->stamp.nsec,
            n2->stamp.sec, n2->stamp.nsec,
            es.N, es.mean_, es.max_);

    *point_error_stream << buff << endl;



  }
  // if (true_dist > 10 ) // cm
  if (false)
  {

    visualization_msgs::MarkerArray m_array;
    for (uint i=0; i<match.inlier.size(); ++i)
    {
      cv::DMatch m = match.inlier[i];

      Eigen::Vector4d p1 = gt_1*n1->frame.pts[m.queryIdx];
      Eigen::Vector4d p2 = gt_2*n2->frame.pts[m.trainIdx];

      visualization_msgs::Marker marker;


      // cerr << "node " << nm_it->second.node_id << " at: " << start << endl;

      marker.header.frame_id = "/openni_rgb_optical_frame";
      marker.header.stamp = ros::Time();
      marker.ns = "inliers_ns";
      marker.id = i;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::MODIFY; // same as add



      geometry_msgs::Point gp,gp2;
      gp.x = p1.array()[0]; gp.y = p1.array()[1];  gp.z = p1.array()[2];
      marker.points.push_back(gp);

      gp2.x = p2.array()[0]; gp2.y = p2.array()[1]; gp2.z = p2.array()[2];
      marker.points.push_back(gp2);

      marker.scale.x = 0.01; // radius in m
      marker.scale.y = 0.03;  // head radius in m

      marker.color.a = 1;

      marker.color.r = marker.color.g = marker.color.b = 0;

      if (m.distance < 10)
        marker.color.g = 1;
      else
        if (m.distance < 25)
          marker.color.b = 1;
        else
          marker.color.r = 1;

      m_array.markers.push_back(marker);
    }

    //  ROS_INFO("sending %i markers", m_array.markers.size());
    pub_marker_array.publish(m_array);


    //    //    ROS_ERROR("true dist: %f", true_dist);
    //    //    cout << "estimated: " << endl << rel_guessed << endl;
    //    //    cout << "correct: " << endl << rel_correct << endl;
    //    //
    //    //    cout << "p1:" << endl << gt_1 << endl;
    //    //    mat2RPY(gt_1.block<3,3>(0,0),r,p,y);
    //    //    printf("rpy: %f %f %f \n", r/M_PI*180,p/M_PI*180,y/M_PI*180 );
    //    //
    //    //    // cout << "p1_inv:" << endl << rel_correct << endl;
    //    //
    //    //
    //    //    cout << "p2:" << endl << gt_2 << endl;
    //    //    mat2RPY(gt_2.block<3,3>(0,0),r,p,y);
    //    //    printf("rpy: %f %f %f \n", r/M_PI*180,p/M_PI*180,y/M_PI*180 );
    //    //
    //    //
    //    //    cout << "est -> correct" << endl << rel_correct.inverse()*rel_guessed << endl;
    //
    // if (pub_new.getNumSubscribers() > 0)
    {

      sensor_msgs::PointCloud2 msg;
      point_cloud foo;
      getTransformed(n1->frame.dense_pointcloud, gt_1, foo);
      pcl::toROSMsg(foo, msg);
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "/openni_rgb_optical_frame";
      pub_new.publish(msg);

      getTransformed(n2->frame.dense_pointcloud, gt_2, foo);
      pcl::toROSMsg(foo, msg);
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "/openni_rgb_optical_frame";
      pub_old.publish(msg);

      //      point_cloud moved;
      //      // getTransformed(n1->frame.dense_pointcloud,gt_1, moved );
      //
      //      pcl::toROSMsg(n1->frame.dense_pointcloud, msg);
      //      msg.header.stamp = ros::Time::now();
      //      msg.header.frame_id = "/openni_rgb_optical_frame";
      //      pub_new.publish(msg);
      //
      //
      //      getTransformed(n2->frame.dense_pointcloud,rel_correct, moved );
      //
      //      pcl::toROSMsg(moved, msg);
      //      msg.header.stamp = ros::Time::now();
      //      msg.header.frame_id = "/openni_rgb_optical_frame";
      //      pub_old.publish(msg);




      // pcl::io::savePCDFileASCII("/home/engelhar/Desktop/test.pcd", moved);
    }

  }


  return true;

}

void checkForForks(Node * new_node, Node* other, vector<cv::DMatch>& m){



  for (uint i=0; i<m.size()-1; ++i)
    for (uint j=i+1; j<m.size(); ++j)
    {
      if (m[i].trainIdx == m[j].trainIdx || m[i].queryIdx == m[j].queryIdx)
      {
        ROS_ERROR("@@@ found fork: %i %i  %i %i", m[i].trainIdx, m[i].queryIdx, m[j].trainIdx, m[j].queryIdx);

        ROS_ERROR("train i: %.0f %.0f",other->frame.kpts[m[i].trainIdx].pt.x,
                  other->frame.kpts[m[i].trainIdx].pt.y);


        ROS_ERROR("query i: %.0f %.0f",new_node->frame.kpts[m[i].queryIdx].pt.x,
                  new_node->frame.kpts[m[i].queryIdx].pt.y);


        ROS_ERROR("train j: %.0f %.0f",other->frame.kpts[m[j].trainIdx].pt.x,
                  other->frame.kpts[m[j].trainIdx].pt.y);


        ROS_ERROR("query j: %.0f %.0f",new_node->frame.kpts[m[j].queryIdx].pt.x,
                  new_node->frame.kpts[m[j].queryIdx].pt.y);

        /* if this happens, either the reprojection error in pose_est was increased
              or the minimal distance between points in the frame_generator was decreased
              Make sure the mindist is more than twice the max error */
        assert(string("One feature has at least two matches in the other image").compare("") == 0);


      }
    }

}

void Cluster_manager::updateLandmarks(Node& new_, Node& other, vector<cv::DMatch>& matches){

  map<uint,uint>::iterator train_it, query_it;


  // ROS_INFO("update landmarks for nodes %i and %i", new_.node_id, other.node_id);


  uint case_new = 0;
  uint case_update = 0;
  uint case_merge = 0;

  for (uint i=0; i<matches.size(); ++i){

    int q_idx = matches[i].queryIdx;
    int t_idx = matches[i].trainIdx;

    query_it =  new_.pt_to_landmark.find(q_idx);
    train_it = other.pt_to_landmark.find(t_idx);

    bool query_has_obs = (query_it != new_.pt_to_landmark.end());
    bool train_has_obs = (train_it != other.pt_to_landmark.end());

    // case 1: new landmark
    if (!query_has_obs && !train_has_obs){

      LM_Observation o_q( new_.node_id,q_idx);
      LM_Observation o_t(other.node_id,t_idx);

      // create new landmark
      Landmark lm;
      lm.id = next_land_mark_id++;
      lm.observations.push_back(o_q);
      lm.observations.push_back(o_t);

      // create new vertex for optimizer
      // use pose of earlier node as basis for pose estimate of landmark

      lm.trackVertex = new g2o::VertexTrackXYZ();
      Eigen::Vector4d p4 = other.pose*other.frame.pts[t_idx];


      Eigen::Vector3d p = Eigen::Vector3d(p4.array()[0],p4.array()[1],p4.array()[2]);
      lm.trackVertex->setEstimate(p);
      //!!! TrackVertex has no id!

      landmarks[lm.id] = lm; // store in cluster_manager

      // store observation also in nodes
      new_.pt_to_landmark[q_idx]  = lm.id;
      other.pt_to_landmark[t_idx] = lm.id;

      //      ROS_INFO("new lm with id %i, n1: %i, pt1: %i, n2: %i, pt2: %i", lm.id, new_.node_id, q_idx, other.node_id, t_idx);
      case_new++;
      continue;
    }

    // case 2: train has seen lm, query not
    if (train_has_obs && !query_has_obs){
      LM_Observation o_q( new_.node_id,q_idx);
      uint lm_id = train_it->second;
      //      ROS_INFO("%i, %i is assigned to lm %i",new_.node_id,q_idx, lm_id );
      new_.pt_to_landmark[q_idx] = lm_id;
      landmarks[lm_id].observations.push_back(o_q);

      case_update++;
      continue;
    }

    // case 3: query has seen lm, train not
    if (query_has_obs && !train_has_obs){


      LM_Observation o_t( other.node_id,t_idx);
      uint lm_id = query_it->second;
      //      ROS_INFO(" Q %i, %i is assigned to lm %i",other.node_id,t_idx, lm_id );
      other.pt_to_landmark[t_idx] = lm_id;
      landmarks[lm_id].observations.push_back(o_t);
      case_update++;
      continue;
    }


    // case 4: both have already a landmark assigned
    if (query_has_obs && train_has_obs){

      case_merge++;

      uint lm_id_q = query_it->second;
      uint lm_id_t = train_it->second;


      // same landmark for both (e.g. comparison 0<->1 and 1<->2, then 0<->2
      if (lm_id_q == lm_id_t) {
        // both nodes are already registered with the landmark
        // ROS_INFO("%i, %i and %i, %i both have lm %i",new_.node_id,q_idx, other.node_id,t_idx, lm_id_q);
        continue;
      }

      //      ROS_INFO("merging landmarks %i and %i", lm_id_q, lm_id_t);

      // insert lm_t to lm_q (wLoG)
      Landmark* lm_to = &landmarks[lm_id_q];
      Landmark lm_from = landmarks[lm_id_t];

      //      ROS_INFO("q(%i) has %i obs, t(%i) has %i",lm_id_q, (int) lm_to->observations.size(), lm_id_t, (int) lm_from.observations.size() );
      //
      //      for (uint o=0; o<lm_from.observations.size(); ++o){
      //        Observation obs = lm_from.observations[o];
      //        ROS_INFO("t: %i %i", obs.node_id, obs.pt_id);
      //      }
      //      for (uint o=0; o<lm_to->observations.size(); ++o){
      //        Observation obs = lm_to->observations[o];
      //        ROS_INFO("q: %i %i", obs.node_id, obs.pt_id);
      //      }


      for (uint o=0; o<lm_from.observations.size(); ++o){
        LM_Observation obs = lm_from.observations[o];

        // add new observation to lm
        lm_to->observations.push_back(obs);

        //        ROS_INFO("%i %i has now lm %i", obs.node_id, obs.pt_id,lm_to->id );

        Node* n = (obs.node_id == new_.node_id)?&new_:&nodes[obs.node_id];

        // check that old observation really pointed to this landmark
        uint old_lm = n->pt_to_landmark[obs.pt_id];


        if (old_lm != lm_id_t){
          ROS_ERROR("fail at landmark merge: old lm: %i, expected: %i", old_lm, lm_id_t);
          assert(1==0);
        }


        // update lm id for this observation
        n->pt_to_landmark[obs.pt_id] = lm_to->id;

      }

      landmarks.erase(lm_id_t);

    }
  }




  //  ROS_INFO("nodes.size(): %i", (int) nodes.size());
  //    ROS_INFO("new (%i) has %i landmarks", new_.node_id,  new_.pt_to_landmark.size());
  //    ROS_INFO("node[%i] has %i landmarks", new_.node_id, nodes[new_.node_id].  pt_to_landmark.size());
  //
  //    ROS_INFO("nodes[1] landmarks:");
  //    for (map<uint, uint>::iterator it = nodes[1].pt_to_landmark.begin(); it != nodes[1].pt_to_landmark.end(); ++it)
  //      ROS_INFO(" %i -> %i", (int) it->first, (int) it->second);

  //  ROS_INFO("update landmark: %i new landmarks, %i adapts, %i merges", case_new, case_update, case_merge);
  //  for (LM_iter it = landmarks.begin(); it != landmarks.end(); ++it)
  //  {
  //    uint lm_id = it->first;
  //    ROS_INFO("lm %i has %i observations", (int) lm_id, (int) it->second.observations.size());
  //
  ////    for (uint i=0; i<it->second.observations.size(); ++i)
  ////    {
  ////      Observation obs = it->second.observations[i];
  ////      Node* n = (obs.node_id == new_.node_id)?&new_:&nodes[obs.node_id];
  ////      uint res = n->pt_to_landmark[obs.pt_id];
  ////      ROS_INFO("lm %i, node %i, pt %i, res: %i", lm_id, obs.node_id, obs.pt_id, res);
  ////      if (res != lm_id){
  ////        ROS_INFO("pt %i in node %i has lm %i", obs.pt_id, obs.node_id, res);
  ////        ROS_ERROR("fail at landmark: expected %i but have %i", lm_id, (int) res);
  ////        assert(1==0);
  ////      }
  ////    }
  //  }



}

bool Cluster_manager::check_pair(Node * new_node, Node* other, Edge_type type, int& inliers,  float* d, float* phi) {



  int64 start = cv::getTickCount();

  // ROS_INFO("node %i is checked against %i",  new_node->node_id, other->node_id);


  // check if comparison was already performed
  map<uint,uint>::iterator it = new_node->comparisons.find(other->node_id);
  if (it != new_node->comparisons.end()){
    //    ROS_INFO("node %i is checked against %i (check was already performed!)",  new_node->node_id, other->node_id);
    return false;
  }

  tested.insert(other->node_id);

  //  ros::Time start = ros::Time::now();
  inliers = pose_estimator_->estimate(new_node->frame,other->frame);
  //  ros::Time end = ros::Time::now();
  //  ROS_INFO("time for estimation: %i ms", (end-start).nsec/1000/1000);


  ROS_INFO("node %i was checked against %i: # %i", new_node->node_id, other->node_id, inliers);


  new_node->comparisons[other->node_id] = inliers;
  other->comparisons[new_node->node_id] = inliers;


  if (all_comparisons != NULL && all_comparisons->is_open())
  {
    (*all_comparisons) << new_node->node_id << "  " << other->node_id << " # " <<  inliers << " t: " << type << endl;
  }



  //  if (type == ET_Keyframe_check)
  //    type = ET_Direct_neighbour;


  // save to file for evaluation:
  // if (new_node->node_id > other->node_id + 20)
  if (match_stream != NULL && match_stream->is_open())
  {

    double r,p,y;

    mat2RPY(pose_estimator_->rot, r,p,y);

    tf::Quaternion rotation = tf::createQuaternionFromRPY(r,p,y);

    // get number of quantized descriptors which are in both images:


    //    ROS_INFO("word sizes: %i, %i",new_node->words.size(), other->words.size());
    vt::Document is(min(new_node->words.size(), other->words.size()));
    vt::Document::iterator end;
    end = set_intersection(new_node->words.begin(),new_node->words.end(),other->words.begin(),other->words.end(), is.begin());


    int word_match_cnt = int(end-is.begin());

    float match_ratio =  word_match_cnt*100.0/min(new_node->words.size(), other->words.size());

    //    ROS_WARN("found %i inlier and %i (%.0f %%) word-matches", inliers,word_match_cnt,match_ratio);


    char buff[200];
    sprintf(buff, "%i, %i, %f,  %i.%i, %i.%i,  %f, %f, %f,   %f, %f, %f, %f",
            inliers,
            word_match_cnt,
            match_ratio,
            new_node->stamp.sec, new_node->stamp.nsec,
            other->stamp.sec, other->stamp.nsec,
            pose_estimator_->trans.array()[0],
            pose_estimator_->trans.array()[1],
            pose_estimator_->trans.array()[2],
            rotation.x(),
            rotation.y(),
            rotation.z(),
            rotation.w());

    *match_stream << buff << endl;
  }


  if (inliers > 3){



    Node_match match;
    match.type = type;
    match.other_id = other->node_id;         // id of other Node
    match.inlier = pose_estimator_->inliers; // inliers
    match.trans = pose_estimator_->trans;    // translation towards other Node
    match.rot = pose_estimator_->rot;        // rotation towards other Node



    // also writes img with all matches
    error_set es;
    EvaluateMatch(new_node, other,match, es);


    // distances are returned even if the match is not valid!
    float d_local = match.trans.norm()*100;
    double r,p,y; mat2RPY(match.rot, r,p,y);
    float max_angle = max(abs(r),max(abs(p),abs(y)));

    if (d != NULL)  *d = d_local;
    if (phi != NULL) *phi = max_angle;


    if (inliers >= min_inlier_cnt) {


      //    Eigen::Matrix4d rel_guessed = combine(match.rot, match.trans);
      //    cout << "trafo " << endl << rel_guessed << endl;




      ROS_INFO("adding node with d = %f", d_local);






      matched_clusters.insert(other->cluster_id);


      cv::Mat img_match;
      cv::drawMatches(new_node->frame.img, new_node->frame.kpts,
                      other->frame.img, other->frame.kpts,
                      match.inlier,img_match, CV_RGB(0,255,0), CV_RGB(0,0,255) );
      cv::namedWindow("matches",1);
      cv::imshow("matches", img_match);
      cv::waitKey(10);


      // assert that no feature has more than one match
      //     checkForForks(new_node, other, match.inlier);


      // update the landmarks
      updateLandmarks(*new_node, *other, match.inlier);


      hits.insert(other->node_id);

      new_node->matches.push_back(match);

      //    ROS_INFO("d,a: %f %f", d_local, max_angle/M_PI*180);

      // symmetric, inverts the transformation and sets new other_node-id
      match.invert(new_node->node_id);
      other->matches.push_back(match);

      if (abs(double(other->node_id-new_node->node_id)) > 10)
        last_matches->add(other->node_id);

    }

  }


  int64 end = cv::getTickCount();

  ticks_for_ransac += (end-start);
  node_comparisons++;

  return (inliers >= min_inlier_cnt);
}

void Cluster_manager::combineCluster(set<uint>& cluster_ids){

  if (cluster_ids.size() < 2)
    return;



  //  merge all clusters into the first one (wlog) [check cluster merging if this behaviour is changed!!]
  uint super_cluster_id = *cluster_ids.begin();
  cluster* super_cluster = &clusters[super_cluster_id];

  // copy all other nodes in the first cluster
  //  for (uint i = 1; i<cluster_ids.size(); ++i){

  for (set<uint>::iterator it = (++cluster_ids.begin()); it != cluster_ids.end();  ++it){
    cluster* cl = &clusters[*it];
    // assign new cluster to each node
    ROS_INFO("copying cluster %i into %i", cl->cluster_id, super_cluster_id);
    for (uint i=0; i<cl->node_ids.size(); ++i)
    {
      uint n_id = cl->node_ids[i];
      nodes[n_id].cluster_id = super_cluster_id;
      super_cluster->node_ids.push_back(n_id);
    }

    super_cluster->changed = true;
    //    clusters.erase(cl->cluster_id);
  }
}

bool Cluster_manager::isNewKeyframe(Node* new_){


  if (keyframe < 0){
    keyframe = new_->node_id;
    new_->is_keyframe = true;
    return true;
  }

  float d=0;
  float phi=0;

  int inlier_cnt;
  /* if the last node matched with a node far away (in time), the new node is also matched
   * against these nodes, even if he becomes no new keyframe */
  set<uint> fh_cp = far_hits;
  far_hits.clear();
  for (set<uint>::iterator it = fh_cp.begin(); it != fh_cp.end(); ++it){
    if (check_pair(new_, &nodes[*it], ET_Far_match,inlier_cnt, &d, &phi)) {
      far_hits.insert(*it);
    }
  }


  //  bool match_last;
  // if (last_added_nodes.size() > 0)

  int cnt = 4;
  //  for (int i = last_added_nodes.size()-1; i>=0 &&  cnt>0; --i,--cnt)
  for (node_map::reverse_iterator it = nodes.rbegin(); it != nodes.rend() && cnt > 0; ++it, --cnt)
  {
    uint id = it->second.node_id;
    //     ROS_INFO("IsKeyFrame: CHECK WITH LAST ADDED: %i", id);

    //uint id = last_added_nodes[i];
    if (int(id) ==  keyframe) continue;

    check_pair(new_, &(nodes[id]), ET_Keyframe_check, inlier_cnt);
  }

  //
  //  if (!match_last)
  //    ROS_ERROR("no connection to last node");
  //  else
  //    ROS_INFO("match with last node");

  inlier_cnt = 0;
  bool keyFrameMatch = check_pair(new_, &nodes[keyframe], ET_Keyframe_check,inlier_cnt, &d, &phi);

  ROS_INFO("check with last keyframe (%i): #%i, %.2f cm %.1f deg", keyframe,inlier_cnt, d, (phi/M_PI*180));

  if (!keyFrameMatch){
    ROS_ERROR("No connection to last keyframe");
    keyframe = new_->node_id;
    new_->is_keyframe = true;
    return true;

    /**
      TODO: compute new position

      - same as last keyframe
      - constant velocity
      - kalman filtered position
      . optimization of last nodes
     */


  }

  if ( (phi/M_PI*180) <= min_rot_deg && d <= min_dist_cm){
    // new node is new keyframe, but still in connected with the last keyframe
    //    uint c_id = nodes[keyframe].cluster_id;
    //    new_->cluster_id = c_id;
    //    clusters[c_id].node_ids.push_back(new_->node_id);

    //    ROS_INFO("node %i is assigned to cluster %i", (int)new_->node_id, c_id );

    new_->is_keyframe = false;
    return false;
  }
  else {
    ROS_INFO("Dist to last keyframe exceeds threshold %.2f cm %.1f deg",d,(phi/M_PI*180));
    new_->is_keyframe = false;
    return true;
  }

}


set<uint> Cluster_manager::addNode_tree_n_last(Node& new_node)
{
  ROS_ERROR("Processing node %i", new_node.node_id);


  hits.clear();
  tested.clear();
  tree_hits.clear();
  tested.insert(new_node.node_id);

  // far_hist is not cleared! if far_hit node is not matched during isCloseToKeyframe,
  // it will be discarded from the list


  // storage for all clusters which match with the new node
  matched_clusters.clear();

  // check with keyframe and eventual far matches,
  // if good connection to last keyframe or close to its position,
  // node is just added to list and tree but not compared to other nodes
  // number of hits for each edge type
  uint hit_neighbour = 0;
  uint hit_ring = 0;
  uint hit_last_match = 0;


  int inlier_cnt;
  if (isNewKeyframe(&new_node)) {
    // also compare with those frames which are not yet in the tree:
    ROS_INFO("Last %i inserted Nodes: ", last_nodes_cnt);
    int cnt = last_nodes_cnt;
    //    for (int i= int(last_added_nodes.size())-1 ; i>=0 && cnt>0 ; --i, --cnt)
    //      if (check_pair(&new_node, &(nodes[last_added_nodes[i]]), ET_Direct_neighbour, inlier_cnt)){ hit_neighbour++; }
    for (node_map::reverse_iterator it = nodes.rbegin(); it != nodes.rend() && cnt > 0; ++it, --cnt){
      if (check_pair(&new_node, &it->second , ET_Direct_neighbour, inlier_cnt)){ hit_neighbour++; }

    }
  }


  //  ROS_INFO("Check with %i last matches", (int) last_matches->v.size());
  //  for (uint i=0; i<last_matches->v.size(); ++i)
  //    if (check_pair(&new_node, &nodes[last_matches->v[i]], ET_Last_match, inlier_cnt )) { hit_last_match++; }


  // find the best fitting frames
  vt::Matches matches;

#ifdef USE_SURF_TREE
  vt_db->find(new_node.words, n_neighbours, matches );
#else
  place_db_->getDocDatabase()->find(new_node.words, n_neighbours, matches );
#endif

  ROS_INFO("Tree Recommendations: ");
  for (uint i=0; i < matches.size() ; ++i){

    map<uint,uint>::iterator it = treeId2NodeId.find(matches.at(i).id);

    if (it == treeId2NodeId.end())
      continue;

    int id = it->second;// treeId2NodeId[matches.at(i).id];
    new_node.tree_proposals.push_back(id);

    if (check_pair(&new_node, &(nodes[id]), ET_Tree_Proposal, inlier_cnt)) { tree_hits.insert(id); }
    ROS_INFO("tree: %i, inls: %i", id, inlier_cnt);
  }


  ROS_INFO("search");
  if (search_depth > 0 && tree_hits.size() > 0)
  {
    set<uint> ring; // all nodes with graph distance <= N from match
    set<uint> blacklist = tested;

    // also check for far-hits?

    for (set<uint>::iterator it = tree_hits.begin(); it != tree_hits.end(); ++it)
      //  for (set<uint>::iterator it = hits.begin(); it != hits.end(); ++it)
      depthSearch(*it,search_depth , &ring, &blacklist);

    //  ROS_INFO("Check with %i-Ring: ", search_depth);

    for (set<uint>::iterator it = ring.begin(); it != ring.end(); ++it){
      //        ROS_INFO("Ring: %i", *it);
      if (check_pair(&new_node, &(nodes[*it]), ET_Ring, inlier_cnt)) { hit_ring++; }
    }
  }


  // save matches with long distance:
  // dist as new parameter:
  //  ROS_INFO("HIT list");
  for (set<uint>::iterator it = hits.begin(); it != hits.end(); ++it){
    if ( new_node.node_id > (*it + 100) ) {far_hits.insert(*it); }
  }



  //} // end of comparison with nodes


  ROS_ERROR("hits: last_n: %i, last_matches: %i, tree: %i, ring: %i, far: %i , total: %i",
            hit_neighbour, hit_last_match, (int) tree_hits.size(), hit_ring, (int) far_hits.size(),
            int(hits.size()));


  ROS_INFO("node %u matched with %i nodes", new_node.node_id, (int) new_node.matches.size());
  int best_node = -1;
  uint best_inlier = 0;
  for (uint i=0; i<new_node.matches.size(); ++i) {
    ROS_INFO("with node %i from cluster %i", new_node.matches[i].other_id, nodes[new_node.matches[i].other_id].cluster_id);
    //    ROS_INFO("inlier: %i, max: %i",new_node.matches[i].getInlierCount(), best_inlier );

    if (new_node.matches[i].getInlierCount() > best_inlier){
      best_inlier=new_node.matches[i].getInlierCount();
      best_node = i;
    }
  }

  //  ROS_INFO("best_node: %i", best_node);

  if (best_node >= 0){

    Node_match* nm = &new_node.matches[best_node];



    Eigen::Matrix4d trafo = combine(nm->rot, nm->trans);

    Node* other = &nodes[nm->other_id];

    new_node.pose = other->getPose()*trafo.inverse();
    //    ROS_ERROR("attaching new Node to node %i", nm->other_id);
    //    cout << "new pose" << endl << new_node.pose << endl;


  }
  else {
    cluster_optimization::setPoseWithConstantvelocity(*this, new_node.node_id);
    //    ROS_ERROR("No neighbour, constant vel");
    //    cout << "new pose" << endl << new_node.pose << endl;
  }



  ROS_INFO("the following %i clusters matched", (int) matched_clusters.size());


  for (set<uint>::iterator it = matched_clusters.begin(); it != matched_clusters.end(); ++it){
    ROS_INFO("cluster %i", (int) *it);
  }


  // wenn ein Knoten an einem Cluster haengt, der noch klein ist, wird noch mit den letzten 10Knoten verglichen
  // TODO: vll auch noch mit den letzten Knoten aus dem letzten Cluster?
  // damit soll nach einem Abbruch verhindert werden, dass sich ein neuer Cluster bildet, ohne das versucht wird,
  // noch an den letzten Cluster zu bilden (vergleich mit frueheren Knoten sonst nur bei nicht-keyframes)


  int node_cnt_in_matches_clusters = 0;
  for (set<uint>::iterator it = matched_clusters.begin(); it != matched_clusters.end(); ++it){
    node_cnt_in_matches_clusters += clusters[*it].node_ids.size();
  }
  ROS_WARN("matched clusters have %i nodes", node_cnt_in_matches_clusters );

  if (node_cnt_in_matches_clusters < 10)
  {
    // Node matched with keyframe
    // Wenn der Knoten einer der ersten in einem neuem Cluster ist, wird er mit den letzten Knoten verglichen, auch wenn
    // mit dem letzten Keyframe gematcht hat.

    int cnt = 20 ;

    ROS_WARN("node is part of small cluster, checking last %i neighbours", cnt);

    for (node_map::reverse_iterator it = nodes.rbegin(); it != nodes.rend() && cnt > 0; ++it, --cnt){
      if (check_pair(&new_node, &it->second , ET_Direct_neighbour, inlier_cnt)){
        ROS_WARN("found new match after cluster_size");
        hit_neighbour++;
      }
    }
  }


  // if the new node has matched with any node (during newKeyframe-check or afterwards (in case of new keyframe))
  // the matched clusters are merged
  if (matched_clusters.size() > 0){

    // add node to cluster with lowest id
    new_node.cluster_id = *matched_clusters.begin();
    clusters[new_node.cluster_id].node_ids.push_back(new_node.node_id);

    // nodes from other clusters are inserted into first cluster
    if (matched_clusters.size() > 1){
      //      vector<uint> as_vector(matched_clusters.begin(), matched_clusters.end());
      // merged clusters are NOT deleted
      combineCluster(matched_clusters); // no action if matched_clusters.size() < 2
    }
    // combined clusters are used during clustermerge (new connections between nodes in the clusters
    //     which become close during optimization
    //      // remove merged clusters
    //      for (set<uint>::iterator it = (++matched_clusters.begin()); it != matched_clusters.end();  ++it){
    //        clusters.erase(*it);
    //      }

  }
  else  // Node hasn't found any connection
  {

    //#define IGNORE_UNMATCHED_NODES


#ifdef IGNORE_UNMATCHED_NODES
    if (next_cluster_id == 0){
#endif

      // starting new cluster
      cluster c;
      c.cluster_id = next_cluster_id++;
      c.node_ids.push_back(new_node.node_id);
      new_node.cluster_id = c.cluster_id;

      clusters[c.cluster_id] = c;
      ROS_ERROR("started cluster %i with node %i ######################################################", (int) c.cluster_id, (int) new_node.node_id);

#ifdef IGNORE_UNMATCHED_NODES
    }

    if (next_cluster_id == 1){ // again first node
      appendNode(new_node);
    }

    return matched_clusters;
#endif


  }



  // cluster was changed and should be optimized
  clusters[new_node.cluster_id].changed = true;

  // finally: add node
  appendNode(new_node);

  // size: 0 for new cluster, 1 for normal match, > 1 after cluster merge
  return matched_clusters;
}


void Cluster_manager::storeNode(Node& node){

  ROS_INFO("store node with id %i", node.node_id);

  if (node.node_id == 0){
    cluster c;
    c.cluster_id = 0;
    c.node_ids.push_back(node.node_id);
    node.cluster_id = 0;
    clusters[0] = c;
  }else
  {
    cluster *c = &clusters.begin()->second;
    assert(c->cluster_id == 0);
    node.cluster_id = 0;
    c->node_ids.push_back(node.node_id);

  }

  // appendNode(node);

}


int Cluster_manager::delete_unimportant_clusters(){


  vector<uint> too_small;
  for (clm_it it = clusters.begin(); it !=clusters.end(); ++it){
    if (it->second.node_ids.size() < 5)
      too_small.push_back(it->first);
  }

  for (uint i=0; i<too_small.size(); ++i){
    assert(1==0); // TODO: also delete nodes
    clusters.erase(i);
  }


  return 0;
}




int Cluster_manager::addMatchedWordsToTree(Node* node){

  bool matched[node->frame.kpts.size()];
  // strongest points are added even if they don't match
  for (uint i=0; i<node->frame.kpts.size(); ++i)
    matched[i] = (i<20);

  for (uint i=0; i<node->matches.size(); ++i){
    Node_match nm = node->matches[i];
    for (uint j=0; j<nm.inlier.size(); ++j)
      matched[nm.inlier.at(j).queryIdx] = true;
  }

  int match_cnt = 0;
  vt::Document words;
  for (uint i=0; i<node->frame.kpts.size(); ++i)
    if (matched[i]) {
      match_cnt++;
      words.push_back(node->words[i]);
    }

  ROS_INFO("Adding %i to tree, matched %i of %i keypoints", node->node_id, match_cnt, int(node->frame.kpts.size()));

  if (match_cnt == 0)
    return -1;
#ifdef USE_SURF_TREE
  int doc_id = vt_db->insert(words);
#else
  int doc_id = place_db_->getDocDatabase()->insert(words);
#endif

  return doc_id;

}


void Cluster_manager::appendNode(Node& node)
{

  //  ROS_INFO("adding node with id %i", node.node_id);

  // write infos to file:
  // appendNodeToFile(&node);

  nodes[node.node_id] = node;

  last_added_nodes.push_back(node.node_id);


  // Maybe: replace by ring which returns removed object at insertion
  if (last_added_nodes.size() > max_stasis_cnt){

    Node* n = &nodes[last_added_nodes.at(0)];

    if (n->words.size() == 0){
      ROS_FATAL("appendNode: node has no words");
      assert(1==0);
    }


    //     int doc_id = addMatchedWordsToTree(n);
#ifdef USE_SURF_TREE
    int doc_id = vt_db->insert(n->words);
#else
    int doc_id = place_db_->getDocDatabase()->insert(n->words);
#endif



    if (doc_id > 0){
      // ROS_INFO("Added node %i with %i points to tree_id %i", n->node_id, (int)  n->words.size(), doc_id);

      treeId2NodeId[doc_id] = n->node_id;
    }


    last_added_nodes.erase(last_added_nodes.begin());
  }


}


void Cluster_manager::sendClouds(ros::Publisher pub, Pose_Type pt, string frame){



  if( pub.getNumSubscribers() == 0){
    ROS_ERROR("publisher has no subscriber!!");
    return;
  }


  point_cloud::Ptr cloud(new point_cloud);

  bool only_keyframes = true;


  // keyframes in clusters with size > 3
  vector<uint> published_nodes;

  for (clm_it it = clusters.begin(); it != clusters.end(); ++it){
    // Don't show small cluster:
    if (it->second.node_ids.size() < 30){
      continue;
    }

    cluster* cl = &it->second;

    for (uint i=0; i<cl->node_ids.size(); ++i){
      Node* n = &nodes[cl->node_ids[i]];
      if (n->is_keyframe || !only_keyframes)
        published_nodes.push_back(n->node_id);
    }

  }

  ROS_INFO("publishing %i of %i nodes", (int) published_nodes.size(), (int) nodes.size());

  int pnt_cnt = 0;
  int keyFrame_cnt = 0;
  // number of points
  for (vector<uint>::iterator it = published_nodes.begin(); it!=published_nodes.end(); ++it )
  {
    pnt_cnt += nodes[*it].frame.dense_pointcloud.points.size();
    keyFrame_cnt++;
  }

  cloud->points.reserve(pnt_cnt);

  ROS_INFO("creating cloud with %i points (%i per frame)", pnt_cnt, pnt_cnt/keyFrame_cnt);


  for (vector<uint>::iterator it = published_nodes.begin(); it!=published_nodes.end(); ++it )
  {
    Node* node = &nodes[*it];
    pnt_cnt += node->frame.dense_pointcloud.points.size();

    point_cloud pc_trafo;

    if (pt == PT_ground_truth)
      getTransformed(node->frame.dense_pointcloud, node->ground_truth_pose, pc_trafo);
    if (pt == PT_inital_estimate)
      getTransformed(node->frame.dense_pointcloud, node->opt_init_pose, pc_trafo);
    if (pt == PT_optimized)
      getTransformed(node->frame.dense_pointcloud, node->opt_pose, pc_trafo);


    cloud->points.insert(cloud->points.end(), pc_trafo.points.begin(), pc_trafo.points.end());

  }


  //  point_cloud res;
  //
  //  ROS_INFO("starting grid");
  //  float voxel_size = 0.03;
  //  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  //  vox_grid.setInputCloud (cloud);
  //  vox_grid.setLeafSize (voxel_size,voxel_size,voxel_size);
  //  vox_grid.filter (res);



  //  ROS_INFO("sending cloud with %i points", (int) res.points.size());

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame;
  pub.publish(msg);


}



void Cluster_manager::getTransformed(const point_cloud& in, const Eigen::Matrix4d&  trafo, point_cloud& out){

  // out.points.resize(in.points.size());

  Eigen::Vector4d p = Eigen::Vector4d::Ones();
  point_type pt;
  for (uint i=0; i<in.points.size(); ++i){
    pt = in.points.at(i);
    p.array()[0] = pt.x;
    p.array()[1] = pt.y;
    p.array()[2] = pt.z;

    p = trafo*p;

    pt.x = p.array()[0];
    pt.y = p.array()[1];
    pt.z = p.array()[2];

    out.points.push_back(pt);
  }
};

//bool Cluster_manager::appendNodeToFile(Node* node){
//
// // ofstream os; os.open(filename.c_str());
//// if (!os.is_open())
////  return false;
//
//// node_map::iterator it;
//// for (it = nodes.begin(); it != nodes.end(); ++it){
////  Node* n = &it->second;
//  for (uint i=0; i<node->matches.size(); ++i){
//   Node_match* nm = &node->matches[i];
//   if (nm->other_id < node->node_id)
//   (*match_stream) << node->node_id << "  " << nm->other_id << "  " << nm->inlier.size() << endl; // "   " << nm->type << endl;
//  }
//
//
//return true;
//}
