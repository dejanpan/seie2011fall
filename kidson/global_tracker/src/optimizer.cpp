/*
 * optimizer.cpp
 *
 *  Created on: Jul 21, 2011
 *      Author: engelhar
 */

#include "cluster_manager.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// #define USE_GROUNDTRUTH_AS_CLUSTER_START
// #define USE_END_OF_LAST_CLUSTER
#define USE_CONSTANT_VELOCITY_MODEL



void cluster_optimization::initOptimizer(g2o::SparseOptimizer* optimizer, frame_common::CamParams cam){



//  g2o::LinearSolverCSparse / g2o::LinearSolverCholmod

  optimizer->setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
  g2o::BlockSolverX::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>(); // alternative: CHOLMOD
  g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(optimizer,linearSolver);
  optimizer->setSolver(solver_ptr);
  optimizer->setVerbose(true);
  optimizer->setUserLambdaInit(100.);

  g2o::CameraParameters* cameraParams = new g2o::CameraParameters();
  cameraParams->setKcam(cam.fx, cam.fy, cam.cx, cam.cy);
  g2o::SE3Quat offset; // identity
  cameraParams->setOffset(offset);
  cameraParams->setId(0);
  optimizer->addParameters(cameraParams);

}




void addEdge(g2o::SparseOptimizer* optimizer, Node* current, Node* other){

  g2o::EdgeSE3 *motionEdge = new g2o::EdgeSE3();
  motionEdge->vertices()[0] = current->camVertex;
  motionEdge->vertices()[1] = other->camVertex;
  motionEdge->information()=Eigen::Matrix<double, 6, 6>::Identity()*0.001;

  Eigen::Matrix4d trans = current->ground_truth_pose.inverse()*other->ground_truth_pose;

  //  ROS_INFO("edge between %i and %i:", current->node_id, other->node_id);
  //  cout << trans << endl;
  //  cout << "current " << current->node_id << endl << current->ground_truth_pose << endl;
  //  cout << "other " << other->node_id << endl << other->ground_truth_pose << endl;

  g2o::SE3Quat new2old = g2o::SE3Quat(trans.topLeftCorner<3,3>(), trans.block<3,1>(0,3));


  motionEdge->setMeasurement(new2old);
  optimizer->addEdge(motionEdge);
}

void addEdge(Cluster_manager& cm, g2o::SparseOptimizer* optimizer, int this_id, int other_id){

  Node* current = &cm.nodes[this_id];
  Node* other  = &cm.nodes[other_id];

  if (!current->ground_truth_valid || !other->ground_truth_valid)
    return;

  addEdge(optimizer, current, other);
}


int cluster_optimization::addEdgesForSoloNodes(Cluster_manager& cm, g2o::SparseOptimizer* optimizer, set<uint>& ids_set){

  vector<uint> ids;
  for (set<uint>::iterator it = ids_set.begin(); it != ids_set.end(); ++it)
    ids.push_back(*it);

  assert(ids.size() > 0);
  sort(ids.begin(), ids.end());

  int step = 5; // dist to neighbours
  int cnt = 2;  // connections in both directions

  // find unconnected nodes with tracker-pose
  for (uint i=0; i<ids.size()-1; i++){
    Node* current = &cm.nodes[ids[i]];
    if (! current->ground_truth_valid || current->isConnected())
      continue;


    char buffer[400];
    sprintf(buffer, "/home/engelhar/Desktop/current_run_output/unmatched_%i.jpg",int(current->node_id));
    for (uint i=0; i<current->frame.kpts.size(); ++i){
      cv::circle(current->frame.img,current->frame.kpts[i].pt, 3, CV_RGB(0,255,0), 1);
    }
    cv::imwrite(buffer, current->frame.img);






    ROS_INFO("adding local edges for solo node %i", current->node_id);

    // connect with nodes in neighbourhood
    //    uint n = current->node_id - cnt*step;
    for (int n = int(current->node_id - cnt*step); n <= int(current->node_id + cnt*step); n+=step ){

      if (n== int(current->node_id)) continue;

      //      ROS_INFO("trying to connect to node %i", n);

      if (ids_set.find(n) == ids_set.end()) continue;

      Node* other = &cm.nodes[n];

      if (!other->ground_truth_valid) continue;


      //      ROS_INFO("connected to node %i", other->node_id);
      addEdge(optimizer, current, other);

    }


  }

  return 0;

}

int cluster_optimization::addConsecutiveEdges(Cluster_manager& cm, g2o::SparseOptimizer* optimizer, set<uint>& ids_set){
  vector<uint> ids;
  for (set<uint>::iterator it = ids_set.begin(); it != ids_set.end(); ++it)
    ids.push_back(*it);

  assert(ids.size() > 0);
  sort(ids.begin(), ids.end());

  for (uint i=0; i<ids.size()-1; i++){

    addEdge(cm,optimizer, ids[i], ids[i+1]);


    //    Node* current = &cm.nodes[ids[i]];
    //    Node* other  = &cm.nodes[ids[i+1]];
    //
    //    g2o::EdgeSE3 *motionEdge = new g2o::EdgeSE3();
    //    motionEdge->vertices()[0] = current->camVertex;
    //    motionEdge->vertices()[1] = other->camVertex;
    //    motionEdge->information()=Eigen::Matrix<double, 6, 6>::Identity()*0.001;
    //
    //    Eigen::Matrix4d trans = current->ground_truth_pose.inverse()*other->ground_truth_pose;
    //
    //    g2o::SE3Quat new2old = g2o::SE3Quat(trans.topLeftCorner<3,3>(), trans.block<3,1>(0,3));
    //
    //    motionEdge->setMeasurement(new2old);
    //    optimizer->addEdge(motionEdge);
  }



  return 0;
}

int cluster_optimization::addTrackerEdges(Cluster_manager& cm, g2o::SparseOptimizer* optimizer, set<uint>& ids_set){


  vector<uint> ids;
  for (set<uint>::iterator it = ids_set.begin(); it != ids_set.end(); ++it)
    ids.push_back(*it);

  assert(ids.size() > 0);
  sort(ids.begin(), ids.end());

  // for every step node, insert edges to nodes with ids i+k*jump
  int step = 10;
  int jump = 9;

  int edge_cnt = 0; // number of added edges

  int edges = 3; // max number of out going edges for a node

  for (uint i=0; i<ids.size(); i+=step){

    //    ROS_INFO("i: %i, id: %i", i, ids[i]);
    int e = 0;
    Node* current = &cm.nodes[ids[i]];

    if (!current->ground_truth_valid){
      //      ROS_INFO("no groundtruth for node %i", current->node_id);
      continue;
    }

    for (uint j = i+jump; j<ids.size() && e < edges; j+=jump, e++){

      //      ROS_INFO("j: %i, id: %i", j, ids[j]);

      Node* other = &cm.nodes[ids[j]];

      if (!other->ground_truth_valid){
        //           ROS_INFO("no groundtruth for node %i", other->node_id);
        continue;
      }

      ROS_INFO("Adding edge between %i and %i t: %f %f", ids[i], ids[j], current->stamp.toSec(), other->stamp.toSec());

      addEdge(optimizer,current, other);

      //      g2o::EdgeSE3 *motionEdge = new g2o::EdgeSE3();
      //      motionEdge->vertices()[0] = current->camVertex;
      //      motionEdge->vertices()[1] = other->camVertex;
      //      motionEdge->information()=Eigen::Matrix<double, 6, 6>::Identity()*0.001;
      //
      //      Eigen::Matrix4d trans = current->ground_truth_pose.inverse()*other->ground_truth_pose;
      //
      //      //      cout << "trafo: " << trans << endl;
      //      //      cout << "current: " << current->ground_truth_pose << endl;
      //      //      cout << "other: " << other->ground_truth_pose << endl;
      //
      //
      //
      //      // Eigen::Matrix4d trans = other->ground_truth_pose.inverse()*current->ground_truth_pose;
      //
      //      g2o::SE3Quat new2old = g2o::SE3Quat(trans.topLeftCorner<3,3>(), trans.block<3,1>(0,3));
      //
      //      motionEdge->setMeasurement(new2old);
      //      optimizer->addEdge(motionEdge);

      edge_cnt++;
    }
  }

  return edge_cnt;
}


void cluster_optimization::populateOptimizer(Cluster_manager& cm, cluster* cl, g2o::SparseOptimizer* optimizer, uint& vertexCounter, set<uint>& optimized_nodes, bool fix_first_in_cluster){


  uint cluster_id = cl->cluster_id;

  // set of landmarks which are seen by nodes in this cluster
  set<uint> seen_landmarks;


  // int fixed_cnt = 0;

  uint node_cnt = cl->node_ids.size();
  for (uint i=0; i<node_cnt; ++i){

    //    ROS_INFO("node: %u",cl->node_ids[i] );
    Node* n = &cm.nodes[cl->node_ids[i]];


    optimized_nodes.insert(n->node_id);

    //if (n->camVertex == NULL){
    //  ROS_INFO("creating camVertex for node %u", n->node_id);
    n->camVertex = new g2o::VertexSE3();
    //}

    g2o::SE3Quat camPose;

    Eigen::Matrix4d pose = n->getPose();

    camPose = g2o::SE3Quat(pose.topLeftCorner<3,3>(), pose.block<3,1>(0,3));


    g2o::SE3Quat robotPose = camPose; // no relative trafo

    n->camVertex->setEstimate(robotPose);


    //    int fixed_cnt;
    //    if (node_cnt > 5)
    //      fixed_cnt = node_cnt-5;
    //    else
    //      fixed_cnt = 0;
    //
    //    n->is_fixed = (i < fixed_cnt && n->was_optimized);
    //
    //
    //    if (n->is_fixed) fixed_cnt++;

    n->is_fixed = fix_first_in_cluster && (i == 0);

    n->camVertex->setFixed(n->is_fixed);
    n->camVertex->setId(vertexCounter++);

    // ROS_INFO("added vertex with id %i", n->node_id);
    optimizer->addVertex(n->camVertex, NULL);


    // get landmarks:
    for (map<uint,uint>::iterator it = n->pt_to_landmark.begin(); it != n->pt_to_landmark.end(); ++it){
      seen_landmarks.insert(it->second);
    }


  }

  //  ROS_INFO("%i nodes are fixed", fixed_cnt);

  //  ROS_INFO("found %i landmarks", int(seen_landmarks.size()));

  uint mean_obs_cnt = 0;

  uint lm_cnt = seen_landmarks.size();
  uint lm_single_cnt = 0;
  // Add Landmarks to graph
  for (set<uint>::iterator it = seen_landmarks.begin(); it!=seen_landmarks.end(); ++it)
  {
    Landmark lm = cm.landmarks[*it];

    // get inital pose of landmark:
    // observation of fixed node or mean if no node is fixed
    Eigen::Vector4d p4(0,0,0,1);
    Eigen::Vector3d p;


    // chose one observation arbitrarily
    uint n_id = lm.observations[0].node_id;
    uint pt_id = lm.observations[0].pt_id;

    //      ROS_INFO("lm %i   %i %i with %i obs", lm.id, n_id, pt_id, lm.observations.size());

    p4 = cm.nodes[n_id].opt_pose*cm.nodes[n_id].frame.pts[pt_id];
    p  = Eigen::Vector3d(p4.array()[0],p4.array()[1],p4.array()[2]);

    // create new trackvertex
    lm.trackVertex = new g2o::VertexTrackXYZ();
    lm.trackVertex->setEstimate(p);
    // end of creation


    lm.trackVertex->setId(vertexCounter++);
    optimizer->addVertex(lm.trackVertex,NULL);

    mean_obs_cnt+= lm.observations.size();

    if (lm.observations.size() == 2)
      lm_single_cnt++;
    // ROS_INFO("lm %u has %i obs", lm.id, (int) lm.observations.size());

    // add observations of this landmark
    for (uint o=0; o < lm.observations.size();  ++o){


      if (optimized_nodes.find(lm.observations[o].node_id) == optimized_nodes.end()){
        continue;
      }


      g2o::EdgeProjectDisparity *projectionEdge = new g2o::EdgeProjectDisparity();
      Node* n = &cm.nodes[lm.observations[o].node_id];


      uint pt_id = lm.observations[o].pt_id;

      //        ROS_INFO("was seen in %i %i", n->node_id, pt_id);

      projectionEdge->setMeasurement(
          Eigen::Vector3d(n->frame.kpts[pt_id].pt.x,
                          n->frame.kpts[pt_id].pt.y,
                          1.0/n->frame.pts[pt_id].z()));

      projectionEdge->setCacheId(0,0);

      projectionEdge->vertices()[0] = n->camVertex;
      projectionEdge->vertices()[1] = lm.trackVertex;
      projectionEdge->setRobustKernel(true); // comparison?
      optimizer->addEdge(projectionEdge);

    }

  }


  // cam-cam edges
  for (set<uint>::iterator it = optimized_nodes.begin(); it != optimized_nodes.end(); ++it){
    Node* n = &cm.nodes[*it];

    for (uint i=0; i<n->matches.size(); ++i){
      Node_match* nm = &n->matches[i];

      if (nm->other_id > n->node_id) // all matches are saved twice (there and back again)
        continue;

      if (optimized_nodes.find(nm->other_id) == optimized_nodes.end()){
        ROS_ERROR("found strange observation (optimizer l. 187)");
        continue;
      }


      Node* other_n = &cm.nodes[nm->other_id];


      ROS_ASSERT(other_n->cluster_id == cluster_id);

      g2o::EdgeSE3 *motionEdge = new g2o::EdgeSE3();
      motionEdge->vertices()[0] = n->camVertex;
      motionEdge->vertices()[1] = other_n->camVertex;
      motionEdge->information()=Eigen::Matrix<double, 6, 6>::Identity()*0.001;
      g2o::SE3Quat new2old = g2o::SE3Quat(nm->rot, nm->trans);

      motionEdge->setMeasurement(new2old);
      optimizer->addEdge(motionEdge);

    }
  }


  ROS_INFO("Optimizing %i nodes with %u landmarks (%.1f obs each, %.1f%% with two obs)",
           node_cnt, (int) lm_cnt,
           mean_obs_cnt*1.0/lm_cnt, lm_single_cnt*100.0/lm_cnt);

}


void cluster_optimization::updatePoses(Cluster_manager& cm, g2o::SparseOptimizer* optimizer, set<uint>& optimized_nodes){


  //  ROS_INFO("updating pose of %i nodes", optimized_nodes.size());

  for (set<uint>::iterator it = optimized_nodes.begin(); it != optimized_nodes.end(); ++it){
    //    ROS_INFO("update pose of node %i",  *it);
    Node* n = &cm.nodes[*it];

    //    cout << "gt: " << n->ground_truth_pose << endl;
    //    cout << "cam: " << n->camVertex->estimate().to_homogenious_matrix() << endl;

    g2o::SE3Quat camPose = n->camVertex->estimate();
    n->opt_pose = camPose.to_homogenious_matrix();
    n->was_optimized = true;

  }

}


void cluster_optimization::setPoseWithConstantvelocity(Cluster_manager& cm, uint node_id){

  if (node_id < 2) return;

  Node* current = &cm.nodes[node_id];
  if (current->was_optimized)
    return;

  Node* prevPrev = &cm.nodes[node_id-2];
  Eigen::Matrix4d prev_prev_pose = prevPrev->getPose();

  Node* prev = &cm.nodes[node_id-1];
  Eigen::Matrix4d prev_pose = prev->getPose();

  Eigen::Matrix4d last_movement = prev_pose*prev_prev_pose.inverse();

  current->pose =  prev_pose*prev_prev_pose.inverse()*prev_pose;


  //  ROS_INFO("constant vel model");
  //  cout <<  endl << "prevprev" << prev_prev_pose << endl;
  //  cout <<  endl <<  "prev" << prev_pose << endl;
  //  cout <<  endl <<  "current: " << current->pose << endl;


}


void cluster_optimization::optimize_cluster(Cluster_manager& cm, cluster* cl, int iterations){
  uint vertexCounter = 0;


  //  ROS_ASSERT(cm.isClusterConnected(*cl));
  Node* first = &(cm.nodes[cl->node_ids[0]]);



#ifdef USE_GROUNDTRUTH_AS_CLUSTER_START
  first->opt_pose = first->ground_truth_pose;
  first->was_optimized = true;
  ROS_ERROR("USED GROUNDTRUTH FOR INITIALIZING FIRST NODE (#%i) OF CLUSTER %i", first->node_id, cl->cluster_id );
#endif


#ifdef USE_END_OF_LAST_CLUSTER
  // use pose of last optimized node as initial pose for first node in cluster
  // TODO: use constant velocity model:
  int current_id = cl->node_ids[0];
  for (int prev_id = current_id-1; prev_id>=0; --prev_id){
    Node* prev_node = &(cm.nodes[prev_id]);
    if (prev_node->was_optimized){
      first->opt_pose = prev_node->opt_pose;
      first->was_optimized = true;
      break;
      ROS_INFO("Node %i copied pose from Node %i", current_id, prev_id);
    }
  }
#endif


#ifdef USE_CONSTANT_VELOCITY_MODEL

  // undo optimization (setPose won't work otherwise)
  first->was_optimized = false;
  setPoseWithConstantvelocity(cm, first->node_id);

#endif


  g2o::SparseOptimizer* optimizer = new g2o::SparseOptimizer();
  initOptimizer(optimizer, first->frame.cam);

  set<uint> optimized_nodes;
  populateOptimizer(cm, cl, optimizer, vertexCounter, optimized_nodes );


  char out_file[200];
  sprintf(out_file,"/home/engelhar/Desktop/current_run_output/before_%i.g2o", int(cl->cluster_id));
  optimizer->save(out_file);

  double achi;

  optimizer->initializeOptimization();
  optimizer->computeActiveErrors();
  achi = optimizer->activeChi2();
  std::cerr << "start   chi2= " << achi << " Normalized chi2= "  << achi / optimizer->edges().size() << std::endl;

  // TODO: nur fuer die neuesten Knoten ausfuehren.
  optimizer->computeInitialGuess();

  optimizer->optimize(iterations);
  achi = optimizer->activeChi2();
  std::cerr << "Final   chi2= " << achi << " Normalized chi2= "  << achi / optimizer->edges().size() << std::endl;

  sprintf(out_file,"/home/engelhar/Desktop/current_run_output/after_%i.g2o", int(cl->cluster_id));
  optimizer->save(out_file);

  updatePoses(cm, optimizer, optimized_nodes);


  // save node positions, s.t. the cluster can be evaluated seperately (no problems with cluster-connections)
  char name[200];
  sprintf(name,"/home/engelhar/Desktop/current_run_output/path_estimate_cl%i.txt", cl->cluster_id);
  ofstream off;
  off.open(name);
  cm.appendClusterToFile(*cl, off);
  cout << "wrote path to " << name << endl;

}



void cluster_optimization::optimize_with_tracker(Cluster_manager& cm, int iterations){

  //  ROS_WARN("optimize_with_tracker");

  uint vertexCounter = 0;

  g2o::SparseOptimizer* optimizer = new g2o::SparseOptimizer();

  Node* first_node = &(cm.nodes[0]);
  initOptimizer(optimizer, first_node->frame.cam);

  set<uint> optimized_nodes;

  vector<uint> optimized_cluster;

  for (clm_it it = cm.clusters.begin(); it != cm.clusters.end(); ++it){


    //    // TODO: add edges for trackerposes
    //        if (it->second.node_ids.size() < 5){
    //
    //          //      Cluster* cl = &it->second;
    //          //      for (node_map::iterator it = )
    //          //
    //          continue;
    //
    //        }

    optimized_cluster.push_back(it->second.cluster_id);
    set<uint> ns;
    populateOptimizer(cm, &it->second, optimizer, vertexCounter, ns , false); // false: don't fix first node in cluster
    optimized_nodes.insert(ns.begin(), ns.end());

  }

  //  assert(optimized_nodes.size() == cm.nodes.size());

  ROS_INFO("fixed node %i",*optimized_nodes.begin() );
  cm.nodes[*optimized_nodes.begin()].camVertex->setFixed(true);

  ROS_INFO("ADDING TRACKER edges");
  addTrackerEdges(cm, optimizer, optimized_nodes);
  ROS_INFO("ADDING Consecutive edges");
  addConsecutiveEdges(cm, optimizer, optimized_nodes);
  ROS_INFO("ADDING local edges for solo nodes");
  addEdgesForSoloNodes(cm, optimizer, optimized_nodes);

  char out_file[200];

  sprintf(out_file,"/home/engelhar/Desktop/current_run_output/before_tracker.g2o");
  optimizer->save(out_file);

  double achi;

  optimizer->initializeOptimization();
  optimizer->computeActiveErrors();
  achi = optimizer->activeChi2();
  std::cerr << "start   chi2= " << achi << " Normalized chi2= "  << achi / optimizer->edges().size() << std::endl;

  // TODO: nur fuer die neuesten Knoten ausfuehren.
  optimizer->computeInitialGuess();


  optimizer->computeActiveErrors();
  optimizer->optimize(iterations);
  achi = optimizer->activeChi2();
  std::cerr << "Final   chi2= " << achi << " Normalized chi2= "  << achi / optimizer->edges().size() << std::endl;

  sprintf(out_file,"/home/engelhar/Desktop/current_run_output/after_tracker.g2o");
  optimizer->save(out_file);


  ROS_WARN("optimized %i nodes", (int) optimized_nodes.size());
  updatePoses(cm, optimizer, optimized_nodes);


  // save node positions, s.t. the cluster can be evaluated seperately (no problems with cluster-connections)
  char name[200];
  sprintf(name,"/home/engelhar/Desktop/current_run_output/path_estimate_tracker.txt");
  ofstream off;
  off.open(name);


  for (uint i=0; i<optimized_cluster.size(); ++i){
    //    ROS_INFO("writing cluster %i to file", (int) optimized_cluster[i]);
    cm.appendClusterToFile(cm.clusters[optimized_cluster[i]], off);
  }

  cout << "wrote path to " << name << endl;


}



void cluster_optimization::optimize_with_landmarks(Cluster_manager& cm, int iterations)
{


  //  for (node_it it = cm.nodes.begin(); it!=cm.nodes.end(); ++it){
  //    ROS_INFO("node %i is in cluster %i with pose: ", it->first, it->second.cluster_id);
  //    cout << it->second.getPose() << endl;
  //  }


  for (clm_it it = cm.clusters.begin(); it != cm.clusters.end(); ++it){

    if (it->second.node_ids.size() < 2){

      for (uint i = 0; i<it->second.node_ids.size(); ++i){

#ifdef USE_GROUNDTRUTH_AS_CLUSTER_START

        Node* n = &cm.nodes[it->second.node_ids[i]];

        n->opt_pose = n->ground_truth_pose;
        n->was_optimized = true;
        ROS_ERROR("USED GROUNDTRUTH FOR INITIALIZING FIRST NODE (#%i) OF CLUSTER %i", n->node_id, it->second.cluster_id );
#endif

#ifdef USE_CONSTANT_VELOCITY_MODEL
        ROS_INFO("initializing pose of node %i with constant vel.", it->second.node_ids[i]);
        setPoseWithConstantvelocity(cm, it->second.node_ids[i]);

#endif

      }

      // no optimization for short paths
      continue;
    }

    // ignore unchanged clusters
    if (it->second.changed)
      it->second.changed = false;
    else{
      cout << "cluster " << it->second.cluster_id << " was not changed." << endl;
      continue;
    }


    cluster* cl = &it->second;
    optimize_cluster(cm, cl,iterations);

  }



}

