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

#include <iostream>
#include <sys/time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
//#include <rgbdslam/CloudTransforms.h>
#include "graph_manager.h"
#include "misc.h"
#include "pcl_ros/transforms.h"
#include "pcl/io/pcd_io.h"
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/features2d/features2d.hpp>
#include <QThread>
#include <qtconcurrentrun.h>
#include <QtConcurrentMap> 
#include <QFile>
#include <utility>
#include <fstream>

#include "g2o/math_groups/se3quat.h"
#include "g2o/types/slam3d/edge_se3_quat.h"

#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
//#include "g2o/solvers/pcg/linear_solver_pcg.h"

// joint optimizer
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/transformation_estimation_joint_optimize.h>
#include <pcl/registration/icp_joint_optimize.h>

#include "pointcloud_acquisition.cpp"
#include "rgbdslam/featureMatch.h"
#include "transform_publisher.h"
#include <pcl/features/normal_3d.h>

#include <iostream>

//typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
//typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;
//typedef std::map<int, g2o::VertexSE3*> VertexIDMap;
typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*>     VertexIDMap;
//std::tr1::unordered_map<int, g2o::HyperGraph::Vertex* >
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;


GraphManager::GraphManager(ros::NodeHandle nh) :
    optimizer_(0), 
    latest_transform_(), //constructs identity
    reset_request_(false),
    marker_id(0),
    last_matching_node_(-1),
    batch_processing_runs_(false),
    process_node_runs_(false)
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  // allocating the optimizer
  optimizer_ = new g2o::SparseOptimizer();
  optimizer_->setVerbose(true);
  //SlamLinearSolver* linearSolver = new SlamLinearSolver();
  SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
  //SlamLinearPCGSolver* linearSolver = new SlamLinearPCGSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* solver = new SlamBlockSolver(optimizer_, linearSolver);
  optimizer_->setSolver(solver);

  //optimizer_ = new AIS::HCholOptimizer3D(numLevels, nodeDistance);
  ParameterServer* ps = ParameterServer::instance();
  batch_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(ps->get<std::string>("individual_cloud_out_topic"),
                                                            ps->get<int>("publisher_queue_size"));
  whole_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(ps->get<std::string>("aggregate_cloud_out_topic"),
                                                            ps->get<int>("publisher_queue_size"));
  ransac_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/correspondence_marker", 
                                                                ps->get<int>("publisher_queue_size"));
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/pose_graph_markers",
                                                         ps->get<int>("publisher_queue_size"));

  feature_match_pub = nh.advertise<rgbdslam::featureMatch>("/feature_match_out_topic",
                                                            ps->get<int>("publisher_queue_size"));

  computed_motion_ = tf::Transform::getIdentity();
  init_base_pose_  = tf::Transform::getIdentity();
  base2points_     = tf::Transform::getIdentity();
  timer_ = nh.createTimer(ros::Duration(0.1), &GraphManager::broadcastTransform, this);

  Max_Depth = -1;

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}


//WARNING: Dangerous
void GraphManager::deleteFeatureInformation() {
  ROS_WARN("Clearing out Feature information from nodes");
  for (unsigned int i = 0; i < graph_.size(); ++i) {
    graph_[i]->clearFeatureInformation();
  }
}

GraphManager::~GraphManager() {
  //TODO: delete all Nodes
    //for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
    for (unsigned int i = 0; i < graph_.size(); ++i) {
      Node* to_delete = graph_[i];
      delete to_delete;
    }
    graph_.clear();
    delete (optimizer_);
    ransac_marker_pub_.shutdown();
    whole_cloud_pub_.shutdown();
    marker_pub_.shutdown();
    batch_cloud_pub_.shutdown();

}

void GraphManager::drawFeatureFlow(cv::Mat& canvas, cv::Scalar line_color,
                                   cv::Scalar circle_color){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    if(!ParameterServer::instance()->get<bool>("use_gui")){ return; }
    ROS_DEBUG("Number of features to draw: %d", (int)last_inlier_matches_.size());

    const double pi_fourth = 3.14159265358979323846 / 4.0;
    const int line_thickness = 1;
    const int circle_radius = 6;
    const int CVV_AA = 16;  //was called CV_AA
    if(graph_.size() == 0) {
      ROS_WARN("Feature Flow for empty graph requested. Bug?");
      return;
    } else if(graph_.size() == 1 || last_matching_node_ == -1 ) {//feature flow is only available between at least two nodes
      Node* newernode = graph_[graph_.size()-1];
      cv::drawKeypoints(canvas, newernode->feature_locations_2d_, canvas, cv::Scalar(255), 5);
      return;
    } 

    Node* earliernode = graph_[last_matching_node_];//graph_.size()-2; //compare current to previous
    Node* newernode = graph_[graph_.size()-1];
    if(earliernode == NULL){
      if(newernode == NULL ){ ROS_ERROR("Nullpointer for Node %u", (unsigned int)graph_.size()-1); }
      ROS_ERROR("Nullpointer for Node %d", last_matching_node_);
      last_matching_node_ = 0;
      return;
    } else if(newernode == NULL ){
      ROS_ERROR("Nullpointer for Node %u", (unsigned int)graph_.size()-1);
      return;
    }

    //encircle all keypoints in this image
    //for(unsigned int feat = 0; feat < newernode->feature_locations_2d_.size(); feat++) {
    //    cv::Point2f p; 
    //    p = newernode->feature_locations_2d_[feat].pt;
    //    cv::circle(canvas, p, circle_radius, circle_color, line_thickness, 8);
    //}
    cv::Mat tmpimage = cv::Mat::zeros(canvas.rows, canvas.cols, canvas.type());
    cv::drawKeypoints(canvas, newernode->feature_locations_2d_, tmpimage, circle_color, 5);
    canvas+=tmpimage;
    for(unsigned int mtch = 0; mtch < last_inlier_matches_.size(); mtch++) {
        cv::Point2f p,q; //TODO: Use sub-pixel-accuracy
        unsigned int newer_idx = last_inlier_matches_[mtch].queryIdx;
        unsigned int earlier_idx = last_inlier_matches_[mtch].trainIdx;
        q = newernode->feature_locations_2d_[newer_idx].pt;
        p = earliernode->feature_locations_2d_[earlier_idx].pt;

        double angle;    angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
        double hypotenuse = cv::norm(p-q);
            cv::line(canvas, p, q, line_color, line_thickness, CVV_AA);
        if(hypotenuse > 1.5){  //only larger motions larger than one pix get an arrow tip
            cv::line( canvas, p, q, line_color, line_thickness, CVV_AA );
            /* Now draw the tips of the arrow.  */
            p.x =  (q.x + 4 * cos(angle + pi_fourth));
            p.y =  (q.y + 4 * sin(angle + pi_fourth));
            cv::line( canvas, p, q, line_color, line_thickness, CVV_AA );
            p.x =  (q.x + 4 * cos(angle - pi_fourth));
            p.y =  (q.y + 4 * sin(angle - pi_fourth));
            cv::line( canvas, p, q, line_color, line_thickness, CVV_AA );
        } else { //draw a smaller circle into the bigger one 
            cv::circle(canvas, p, circle_radius-2, circle_color, line_thickness, CVV_AA);
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}


/// max_targets determines how many potential edges are wanted
/// max_targets < 0: No limit
/// max_targets = 0: Compare to first frame only
/// max_targets = 1: Compare to previous frame only
/// max_targets > 1: Select intelligently (TODO: rather stupid at the moment)
QList<int> GraphManager::getPotentialEdgeTargets(const Node* new_node, int max_targets){
    int last_targets = 8; //always compare to the last n, spread evenly for the rest //3
    QList<int> ids_to_link_to;
    //max_targets = last_targets;
    int gsize = graph_.size();
    //Special Cases
    if(gsize == 0){
        ROS_WARN("Do not call this function as long as the graph is empty");
        return ids_to_link_to;
    }
    if(max_targets-last_targets < 0) 
        return ids_to_link_to;
    if(max_targets-last_targets == 0){
        ids_to_link_to.push_back(0);
        return ids_to_link_to; //only compare to first frame
    } else if(max_targets-last_targets == 1){
        ids_to_link_to.push_back(gsize-2); 
        return ids_to_link_to; //only compare to previous frame
    } 
    //End Special Cases
    
    //All the last few nodes
    if(gsize <= max_targets){
      last_targets = gsize;
    }
    for(int i = 2; i <= gsize && i <= last_targets; i++){//start at two, b/c the prev node is always already checked in addNode{
        ids_to_link_to.push_back(gsize-i);
    }
    while(ids_to_link_to.size() < max_targets && ids_to_link_to.size() < gsize-1){
        int sample_id = rand() % (gsize - 1);
        ROS_DEBUG_STREAM("Sample: " << sample_id << " Graph size: " << gsize << " ids_to_link_to.size: " << ids_to_link_to.size());
        //usleep(100000);
        QList<int>::const_iterator i1 = qFind(ids_to_link_to, sample_id);
        if(i1 != ids_to_link_to.end()) 
          continue;
        ids_to_link_to.push_back(sample_id);
    }
    //if((gsize - 5) > 0)
    //    	ids_to_link_to.push_back(gsize - 5);
    //if((gsize - 20) > 0)
    //    	ids_to_link_to.push_back(gsize - 15);



    //output only loop
    std::stringstream ss;
    ss << "Node ID's to compare with candidate for node " << graph_.size() << ":";
    for(int i = 0; i < (int)ids_to_link_to.size(); i++){
        ss << ids_to_link_to[i] << ", " ; 
    }
    ROS_INFO("%s", ss.str().c_str()); //print last one here, to avoid trailing ","
    return ids_to_link_to;
}
///// max_targets determines how many potential edges are wanted
///// max_targets < 0: No limit
///// max_targets = 0: Compare to first frame only
///// max_targets = 1: Compare to previous frame only
///// max_targets > 1: Select intelligently (TODO: rather stupid at the moment)
//QList<int> GraphManager::getPotentialEdgeTargets(const Node* new_node, int max_targets){
//    const int last_targets = 3; //always compare to the last n, spread evenly for the rest
//    QList<int> ids_to_link_to;
//    double max_id_plus1 = (double)graph_.size()- last_targets;
//    max_targets -= last_targets;
//    //Special Cases
//    if(graph_.size() == 0){
//        ROS_WARN("Do not call this function as long as the graph is empty");
//        return ids_to_link_to;
//    }
//    if(max_targets < 0) 
//        return ids_to_link_to;
//    if(max_targets == 0){
//        ids_to_link_to.push_back(0);
//        return ids_to_link_to; //only compare to first frame
//    } else if(max_targets == 1){
//        ids_to_link_to.push_back(graph_.size()-2); 
//        return ids_to_link_to; //only compare to previous frame
//    } 
//    
//    //Subset of the majority of the  nodes
//    double id = 0.0; //always start with the first
//    double increment = max_id_plus1/(double)(max_targets);//max_targets-1 = intermediate steps
//    increment = increment < 1.0 ? 1.0 : increment; //smaller steps would select some id's twice
//    ROS_DEBUG("preparing loop %f %f", id, increment);
//    while((int)id < (int)max_id_plus1){
//        ids_to_link_to.push_back((int)id); //implicit rounding
//        id += increment;
//    }
//    //All the last few nodes
//    for(int i = 2; i <= (int)graph_.size() && i <= last_targets; i++){//start at two, b/c the prev node is always already checked in addNode{
//        ids_to_link_to.push_back(graph_.size()-i);
//    }
//    //output only loop
//    std::stringstream ss;
//    ss << "Node ID's to compare with candidate for node " << graph_.size() << ":";
//    for(int i = 0; i < (int)ids_to_link_to.size()-1; i++){
//        ss << ids_to_link_to[i] << ", " ; 
//    }
//    ROS_DEBUG("%s%i", ss.str().c_str(), ids_to_link_to.last()); //print last one here, to avoid trailing ","
//    return ids_to_link_to;
//}

// This function retrieves node ids that are not connected to the given node (for 2nd stage optimization)

QList<int> GraphManager::getUnconnectedNodes(const Node* new_node, int max_targets){
//typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;
//typedef std::vector<Vertex*>                      VertexVector;

	std::vector<int> connectedNodes;
	EdgeSet nodeEdges = optimizer_->vertex(new_node->id_)->edges();
	// for a given node, find all connected nodes
	for(EdgeSet::iterator iterator = nodeEdges.begin(); iterator != nodeEdges.end(); iterator++)
	{
		g2o::HyperGraph::VertexVector edgeVertices = (**iterator).vertices();
		if(edgeVertices[0]->id() != new_node->id_)
			connectedNodes.push_back(edgeVertices[0]->id());
		else if(edgeVertices[1]->id() != new_node->id_)
			connectedNodes.push_back(edgeVertices[1]->id());
	}
	for(size_t i = 0; i < connectedNodes.size(); i++)
		ROS_INFO_STREAM("node[" << new_node->id_ << "] connectedNode[" << i << "] : " << connectedNodes[i]);
    QList<int> ids_to_link_to;
    // iterate through the graph, rejecting nodes already connected.
    for(size_t i=(new_node->id_+1); i < graph_.size(); i++)  // don't check previous nodes to avoid overlap
    {
    	//check if new_node->id_ is connected to graph_[i]
    	bool alreadyConnected = false;
        for(size_t j=0; j < connectedNodes.size(); j++)
        {
        	if(graph_[i]->id_ == connectedNodes[j])
        		alreadyConnected = true;
        }
        if(alreadyConnected)
        	continue;

        //check the transformation difference between nodes.  Reject if too high
		if (!isTrafoSmall(getGraphTransformBetweenNodes(new_node->id_, i)))	// nodes are too far away to do rgbdicp
		{
			ROS_INFO_STREAM("trafo too small. trafo: " << getGraphTransformBetweenNodes(new_node->id_, i));
			continue;
		}

        // add node to check
    	ids_to_link_to.push_back(i);
    	if(ids_to_link_to.size() >= max_targets)	//if limit is reached
    		return ids_to_link_to;
    }
	for(size_t i = 0; i < ids_to_link_to.size(); i++)
		ROS_INFO_STREAM("node[" << new_node->id_ << "] ids_to_link_to[" << i << "] : " << ids_to_link_to[i]);

    return ids_to_link_to;
}

Eigen::Matrix4f GraphManager::getGraphTransformBetweenNodes(const int sourceId, const int targetId)
{
	if(sourceId < 0 || targetId < 0)
		ROS_ERROR("Invalid node id!");
	g2o::VertexSE3* sourceVertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(sourceId));
	g2o::VertexSE3* targetVertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(targetId));
	if(!sourceVertex || !targetVertex)
		ROS_ERROR("Nullpointer in graph at position!");

	Eigen::Matrix4f source = g2o2EigenMat(sourceVertex->estimate());
	Eigen::Matrix4f target = g2o2EigenMat(targetVertex->estimate());
	return (target.inverse() * source);
}

void GraphManager::resetGraph(){
    marker_id =0;
    delete optimizer_; 
    // allocating the optimizer
    optimizer_ = new g2o::SparseOptimizer();
    //SlamLinearSolver* linearSolver = new SlamLinearSolver();
    SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* solver = new SlamBlockSolver(optimizer_, linearSolver);
    optimizer_->setSolver(solver);

    graph_.clear();//TODO: also delete the nodes
    Q_EMIT resetGLViewer();
    last_matching_node_ = -1;
    latest_transform_.setToIdentity();
    current_poses_.clear();
    current_edges_.clear();
    reset_request_ = false;

}

// returns true, iff node could be added to the cloud
bool GraphManager::addNode(Node* new_node) {
    /// \callergraph
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    process_node_runs_ = true;

    last_inlier_matches_.clear();
    if(reset_request_) resetGraph(); 

    if (new_node->feature_locations_2d_.size() <= 50){
        ROS_DEBUG("found only %i features on image, node is not included",(int)new_node->feature_locations_2d_.size());
        process_node_runs_ = false;
        return false;
    }

    //set the node id only if the node is actually added to the graph
    //needs to be done here as the graph size can change inside this function
    new_node->id_ = graph_.size();
    ROS_DEBUG("New Node with id %i has address %p (GraphManager is %p)", new_node->id_, new_node, this);

    //First Node, so only build its index, insert into storage and add a
    //vertex at the origin, of which the position is very certain
    if (graph_.size()==0){
        init_base_pose_ =  new_node->getGroundTruthTransform();//identity if no MoCap available
        new_node->buildFlannIndex(); // create index so that next nodes can use it
        graph_[new_node->id_] = new_node;
        new_node->cachePointCloudToFile();
        new_node->clearPointCloud();
        g2o::VertexSE3* reference_pose = new g2o::VertexSE3;
        reference_pose->setId(0);
        reference_pose->setEstimate(g2o::SE3Quat());
        reference_pose->setFixed(true);//fix at origin
        optimizer_mutex.lock();
        optimizer_->addVertex(reference_pose); 
        optimizer_mutex.unlock();
        QString message;
        Q_EMIT setGUIInfo(message.sprintf("Added first node with %i keypoints to the graph", (int)new_node->feature_locations_2d_.size()));
        pointcloud_type::Ptr the_pc = new_node->pc_col;
        Q_EMIT setPointCloud(the_pc.get(), latest_transform_);
        current_poses_.append(latest_transform_);
        ROS_DEBUG("GraphManager is thread %d, New Node is at (%p, %p)", (unsigned int)QThread::currentThreadId(), new_node, graph_[0]);
        process_node_runs_ = false;
        return true;
    }

    /*
    g2o::HyperDijkstra hypdij(optimizer_);
    g2o::UniformCostFunction f;
    g2o::VertexSE3* root_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(0));
    hypdij.shortestPaths(root_vertex,&f);
    */


    unsigned int num_edges_before = optimizer_->edges().size(); 

    ROS_DEBUG("Graphsize: %d", (int) graph_.size());
    marker_id = 0; //overdraw old markers
    last_matching_node_ = -1;


    //MAIN LOOP: Compare node pairs ######################################################################
    //First check if trafo to last frame is not too small
    Node* prev_frame = graph_[graph_.size()-1];
    ROS_INFO("Comparing new node (%i) with previous node %i", new_node->id_, prev_frame->id_);
    MatchingResult mr = new_node->matchNodePair(prev_frame, (unsigned int) ParameterServer::instance()->get<int>("min_matches"));

    if(mr.edge.id1 >= 0 && !isBigTrafo(mr.edge.mean)){
        ROS_WARN("Transformation not relevant. Did not add as Node");
        process_node_runs_ = false;
        return false;
    } else if(mr.edge.id1 >= 0){

    	//code to publish transform data here
    	if(new_node->id_ == 1)
    		publish_transform(mr, new_node, prev_frame, feature_match_pub);
    	// everything after this point (optimizer, etc) is no longer required.
    	// left in because it doesn't seem to affect performance and it may be useful to compare with
    	// rgbd_icp optimizer

        //mr.edge.informationMatrix *= geodesicDiscount(hypdij, mr); 
        ROS_DEBUG_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << "\n" << mr.edge.informationMatrix);

        if (addEdgeToG2O(mr.edge, true, true)) {
            ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
            last_matching_node_ = mr.edge.id1;
            last_inlier_matches_ = mr.inlier_matches;
            last_matches_ = mr.all_matches;
            edge_to_previous_node_ = mr.edge.mean;

        } else {
          process_node_runs_ = false;
          return false;
        }
    }
    //Eigen::Matrix4f ransac_trafo, final_trafo;
    QList<int> vertices_to_comp = getPotentialEdgeTargets(new_node, ParameterServer::instance()->get<int>("connectivity")); //vernetzungsgrad
    QList<const Node* > nodes_to_comp;//only necessary for parallel computation

    if (ParameterServer::instance()->get<bool>("concurrent_edge_construction")) {
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) {
            //First compile a qlist of the nodes to be compared, then run the comparisons in parallel,
            //collecting a qlist of the results (using the blocking version of mapped).
            nodes_to_comp.push_back(graph_[vertices_to_comp[id_of_id]]);
        }
        printMultiThreadInfo("node comparison");
        QList<MatchingResult> results = QtConcurrent::blockingMapped(
                nodes_to_comp, boost::bind(&Node::matchNodePair, new_node, _1, (unsigned int) ParameterServer::instance()->get<int>("min_matches")));

        for (int i = 0; i < results.size(); i++) {
            MatchingResult& mr = results[i];

            if (mr.edge.id1 >= 0) {
                //mr.edge.informationMatrix *= geodesicDiscount(hypdij, mr);
                ROS_INFO_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << "\n" << mr.edge.informationMatrix);

                if (addEdgeToG2O(mr.edge, isBigTrafo(mr.edge.mean),
                        mr.inlier_matches.size() > last_inlier_matches_.size())) { //TODO: result isBigTrafo is not considered
                    ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                    if (mr.inlier_matches.size() > last_inlier_matches_.size()) {
                        last_matching_node_ = mr.edge.id1;
                        last_inlier_matches_ = mr.inlier_matches;
                        last_matches_ = mr.all_matches;
                        //last_edge_ = mr.edge.mean;
                    }
                }
            }
        }
    } else {
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) {
            Node* abcd = graph_[vertices_to_comp[id_of_id]];
            ROS_INFO("Comparing new node (%i) with node %i / %i", new_node->id_, vertices_to_comp[id_of_id], abcd->id_);
            MatchingResult mr = new_node->matchNodePair(abcd, (unsigned int) ParameterServer::instance()->get<int>("min_matches"));

            if (mr.edge.id1 >= 0){
				//if((new_node->id_== 2) || (abcd->id_ == 2))
				//	publish_transform(mr, new_node, abcd, feature_match_pub);
				//mr.edge.informationMatrix *= geodesicDiscount(hypdij, mr);
				ROS_INFO_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << "\n" << mr.edge.informationMatrix);

				if (addEdgeToG2O(mr.edge, isBigTrafo(mr.edge.mean),
						mr.inlier_matches.size() > last_inlier_matches_.size())) { //TODO: result isBigTrafo is not considered
					ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
					if (mr.inlier_matches.size() > last_inlier_matches_.size()) {
						last_matching_node_ = mr.edge.id1;
						last_inlier_matches_ = mr.inlier_matches;
						last_matches_ = mr.all_matches;
						//last_edge_ = mr.edge.mean;
					}
				}
            }
        }
    }

//for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) {
    //#ifndef CONCURRENT_EDGE_COMPUTATION
////#define QT_NO_CONCURRENT
////#endif
////#ifndef QT_NO_CONCURRENT
//
//        //First compile a qlist of the nodes to be compared, then run the comparisons in parallel,
//        //collecting a qlist of the results (using the blocking version of mapped).
//        nodes_to_comp.push_back(graph_[vertices_to_comp[id_of_id]]);
//    }
//    QThreadPool* qtp = QThreadPool::globalInstance();
//    ROS_INFO("Running node comparisons in parallel in %i (of %i) available threads",  qtp->maxThreadCount() - qtp->activeThreadCount(), qtp->maxThreadCount());
//    if( qtp->maxThreadCount() - qtp->activeThreadCount() == 1){
//      ROS_WARN("Few Threads Remaining: Increasing maxThreadCount to %i", qtp->maxThreadCount()+1);
//      qtp->setMaxThreadCount(qtp->maxThreadCount()+1);
//    }
//    QList<MatchingResult> results = QtConcurrent::blockingMapped(nodes_to_comp, boost::bind(&Node::matchNodePair, new_node, _1));
//    for(int i = 0; i <  results.size(); i++){
//        MatchingResult& mr = results[i];
////#else
//        Node* abcd = graph_[vertices_to_comp[id_of_id]];
//        ROS_INFO("Comparing new node (%i) with node %i / %i", new_node->id_, vertices_to_comp[id_of_id], abcd->id_);
//        MatchingResult mr = new_node->matchNodePair(abcd);
////#endif
//        if(mr.edge.id1 >= 0){
//            //mr.edge.informationMatrix *= geodesicDiscount(hypdij, mr);
//            ROS_INFO_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << "\n" << mr.edge.informationMatrix);
//
//            if (addEdgeToG2O(mr.edge, isBigTrafo(mr.edge.mean), mr.inlier_matches.size() > last_inlier_matches_.size())) { //TODO: result isBigTrafo is not considered
//                ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
//                if(mr.inlier_matches.size() > last_inlier_matches_.size()){
//                  last_matching_node_ = mr.edge.id1;
//                  last_inlier_matches_ = mr.inlier_matches;
//                  last_matches_ = mr.all_matches;
//                }
//            }
//        }
//    }
    //END OF MAIN LOOP: Compare node pairs ######################################################################

    if(ParameterServer::instance()->get<bool>("keep_all_nodes")){
      if (optimizer_->edges().size() == num_edges_before) { //Failure: Create Bogus edge
        ROS_WARN("Node %u could not be matched. Adding with constant motion assumption", (unsigned int)graph_.size());
        LoadedEdge3D virtual_edge;
        virtual_edge.id1 = graph_.size()-1;
        virtual_edge.id2 = graph_.size();
        virtual_edge.mean = edge_to_previous_node_;
        latest_transform_ = g2o2QMatrix(edge_to_previous_node_);
        last_matching_node_ = virtual_edge.id1;
        virtual_edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity();// *(1e-6);
        addEdgeToG2O(virtual_edge, true,true) ;
      }
    }
      


    if (optimizer_->edges().size() > num_edges_before) { //Success
        new_node->buildFlannIndex();
        graph_[new_node->id_] = new_node;
        ROS_INFO("Added Node, new Graphsize: %i", (int) graph_.size());
        if(new_node->id_ % ParameterServer::instance()->get<int>("pointcloud_skip_step") == 0)
        	new_node->cachePointCloudToFile();
        new_node->clearPointCloud();
        if((optimizer_->vertices().size() % ParameterServer::instance()->get<int>("optimizer_skip_step")) == 0){ 
          optimizeGraph();
        } else {
          current_poses_.append(latest_transform_);
          current_edges_.append( qMakePair((int)new_node->id_, last_matching_node_));
          Q_EMIT setGraphEdges(new QList<QPair<int, int> >(current_edges_));
          Q_EMIT updateTransforms(new QList<QMatrix4x4>(current_poses_));
        }
        //Q_EMIT updateTransforms(getAllPosesAsMatrixList());
        //Q_EMIT setGraphEdges(getGraphEdges());
        //make the transform of the last node known
        broadcastTransform(ros::TimerEvent());
        visualizeGraphEdges();
        visualizeGraphNodes();
        visualizeFeatureFlow3D(marker_id++);
        //if(last_matching_node_ <= 0){ cloudRendered(new_node->pc_col.get());}//delete points of non-matching nodes. They shall not be rendered

        //The following updates the 3D visualization. Important only if optimizeGraph is not called every frame, as that does the update too
        pointcloud_type::Ptr the_pc = new_node->pc_col;
        Q_EMIT setPointCloud(the_pc.get(), latest_transform_);

        ROS_DEBUG("GraphManager is thread %d", (unsigned int)QThread::currentThreadId());
    }else{ 
        if(graph_.size() == 1){//if there is only one node which has less features, replace it by the new one
          ROS_WARN("Choosing new initial node, because it has more features");
          if(new_node->feature_locations_2d_.size() > graph_[0]->feature_locations_2d_.size()){
            this->resetGraph();
            process_node_runs_ = false;
            return this->addNode(new_node);
          }
        } else { //delete new_node; //is now  done by auto_ptr
          ROS_WARN("Did not add as Node");
        }
    }
    QString message;
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    Q_EMIT setGUIInfo(message.sprintf("%s, Graph Size: %iN/%iE, Duration: %f, Inliers: %i, &chi;<sup>2</sup>: %f", 
                                     (optimizer_->edges().size() > num_edges_before) ? "Added" : "Ignored",
                                     (int)optimizer_->vertices().size(), (int)optimizer_->edges().size(),
                                     elapsed, (int)last_inlier_matches_.size(), optimizer_->chi2()));
    process_node_runs_ = false;
    return (optimizer_->edges().size() > num_edges_before);
}

void GraphManager::printMultiThreadInfo(std::string task)
{
	QThreadPool* qtp = QThreadPool::globalInstance();
	ROS_INFO_STREAM("Running " << task << " in parallel in "
			<< qtp->maxThreadCount()- qtp->activeThreadCount() << " (of " << qtp->maxThreadCount() << ") available threads");
	if (qtp->maxThreadCount() - qtp->activeThreadCount() == 1) {
		ROS_WARN("Few Threads Remaining: Increasing maxThreadCount to %i", qtp->maxThreadCount()+1);
		qtp->setMaxThreadCount(qtp->maxThreadCount() + 1);
	}
}

///Get the norm of the translational part of an affine matrix (Helper for isBigTrafo)
void GraphManager::visualizeFeatureFlow3D(unsigned int marker_id,
                                          bool draw_outlier) const{
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    if (ransac_marker_pub_.getNumSubscribers() > 0){ //don't visualize, if nobody's looking

        visualization_msgs::Marker marker_lines;

        marker_lines.header.frame_id = "/openni_rgb_optical_frame";
        marker_lines.ns = "ransac_markers";
        marker_lines.header.stamp = ros::Time::now();
        marker_lines.action = visualization_msgs::Marker::ADD;
        marker_lines.pose.orientation.w = 1.0;
        marker_lines.id = marker_id;
        marker_lines.type = visualization_msgs::Marker::LINE_LIST;
        marker_lines.scale.x = 0.002;
        
        std_msgs::ColorRGBA color_red  ;  //red outlier
        color_red.r = 1.0;
        color_red.a = 1.0;
        std_msgs::ColorRGBA color_green;  //green inlier, newer endpoint
        color_green.g = 1.0;
        color_green.a = 1.0;
        std_msgs::ColorRGBA color_yellow;  //yellow inlier, earlier endpoint
        color_yellow.r = 1.0;
        color_yellow.g = 1.0;
        color_yellow.a = 1.0;
        std_msgs::ColorRGBA color_blue  ;  //red-blue outlier
        color_blue.b = 1.0;
        color_blue.a = 1.0;

        marker_lines.color = color_green; //just to set the alpha channel to non-zero
        const g2o::VertexSE3* earlier_v; //used to get the transform
        const g2o::VertexSE3* newer_v; //used to get the transform
        VertexIDMap v_idmap = optimizer_->vertices();
        // end of initialization
        ROS_DEBUG("Matches Visualization start: %lu Matches, %lu Inliers", last_matches_.size(), last_inlier_matches_.size());

        // write all inital matches to the line_list
        marker_lines.points.clear();//necessary?

        if (draw_outlier)
        {
            for (unsigned int i=0;i<last_matches_.size(); i++){
                int newer_id = last_matches_.at(i).queryIdx; //feature id in newer node
                int earlier_id = last_matches_.at(i).trainIdx; //feature id in earlier node

                earlier_v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(last_matching_node_));
                newer_v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_.size()-1));

                //Outliers are red (newer) to blue (older)
                marker_lines.colors.push_back(color_red);
                marker_lines.colors.push_back(color_blue);

                Node* last = graph_.find(graph_.size()-1)->second;
                marker_lines.points.push_back(
                        pointInWorldFrame(last->feature_locations_3d_[newer_id], newer_v->estimate()));
                Node* prev = graph_.find(last_matching_node_)->second;
                marker_lines.points.push_back(
                        pointInWorldFrame(prev->feature_locations_3d_[earlier_id], earlier_v->estimate()));
            }
        }

        for (unsigned int i=0;i<last_inlier_matches_.size(); i++){
            int newer_id = last_inlier_matches_.at(i).queryIdx; //feature id in newer node
            int earlier_id = last_inlier_matches_.at(i).trainIdx; //feature id in earlier node

            earlier_v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(last_matching_node_));
            newer_v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_.size()-1));


            //inliers are green (newer) to blue (older)
            marker_lines.colors.push_back(color_green);
            marker_lines.colors.push_back(color_blue);

            Node* last = graph_.find(graph_.size()-1)->second;
            marker_lines.points.push_back(
                    pointInWorldFrame(last->feature_locations_3d_[newer_id], newer_v->estimate()));
            Node* prev = graph_.find(last_matching_node_)->second;
            marker_lines.points.push_back(
                    pointInWorldFrame(prev->feature_locations_3d_[earlier_id], earlier_v->estimate()));
        }

        ransac_marker_pub_.publish(marker_lines);
        ROS_DEBUG_STREAM("Published  " << marker_lines.points.size()/2 << " lines");
    }
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GraphManager::calculateNormals(const pointcloud_type::Ptr pc_col)
{
	pointcloud_type::Ptr pointCloudOut (new pointcloud_type);
	pcl::NormalEstimation<point_type, point_type> ne;
	ne.setInputCloud (pc_col);
	pcl::search::KdTree<point_type>::Ptr tree (new pcl::search::KdTree<point_type> ());
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.03);
	ne.compute (*pc_col);
}

std::vector<int> GraphManager::extractHandles(const pointcloud_type::Ptr pc_col)
{
	std::vector<int> handleIndices;
	handleExtractor_.extractHandles(pc_col, handleIndices);
	// THIS FUNCTION IS MISSING CHECKING FOR NORMALS WITH NANS IF IT IS TO BE USED
	return handleIndices;
}

void GraphManager::runRGBDICPOptimization()
{
	ROS_INFO_STREAM(" GraphManager::runRGBDICPOptimization");

	//make sure the graph is optimized before starting
	optimizeGraph();

	//Calculate normals for all nodes
	QList<pointcloud_type::Ptr> pointcloudList;
	for (unsigned int i = 0; i < graph_.size(); ++i) {
		pointcloudList.push_back(graph_[i]->pc_col);
	}
	printMultiThreadInfo("point cloud normals");
	QtConcurrent::blockingMap(pointcloudList, boost::bind(&GraphManager::calculateNormals, this, _1));
	ROS_INFO("End of point cloud normal calculation");

	//Extract handle indices for all nodes ROSS-TODO:: multithread this like a boss
	pcl::PCDWriter writer;
	for (unsigned int i = 0; i < graph_.size(); ++i) {
		ROS_INFO("Extracting handles for node %i",(int)i);
		graph_[i]->extractHandlesIndices();

		std::vector<int> removedPoints;
		pointcloud_type::Ptr tempCloud (new pointcloud_type);
		removeNaNs(graph_[i]->pc_col, tempCloud, removedPoints);
		pcl::copyPointCloud(*tempCloud, *(graph_[i]->pc_col));
		adjustIndicesFromRemovedPoints(graph_[i]->handleIndices,  removedPoints);

		std::stringstream filename;
		filename << "node_" << graph_[i]->id_ << "_handles.pcd";
		writer.write (filename.str(), *(graph_[i]->pc_col), graph_[i]->handleIndices, true);
	}
//	QList<std::vector<int> > nodeHandleIndices = QtConcurrent::blockingMapped(pointcloudList, boost::bind(&GraphManager::extractHandles, this, _1));
//	ROS_INFO("End of handle extraction multithreading");

    for (unsigned int i = 0; i < graph_.size(); ++i) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
        if(!v){
            ROS_ERROR("Nullpointer in graph at position %i!", i);
            continue;
        }
        if(graph_[i]->pc_col->size() == 0){
            ROS_INFO("Skipping Node %i, point cloud data is empty!", i);
            continue;
        }
        QList<int> nodesToCompareIndices = getUnconnectedNodes(graph_[i], 1000);
        QList<const Node* > nodes_to_comp;// for parallel computation
        QList<MatchingResult> results;

        // put nodes into a list for multithreading
        for (int id_of_id = (int) nodesToCompareIndices.size() - 1; id_of_id >= 0; id_of_id--) {
        	nodes_to_comp.push_back(graph_[nodesToCompareIndices[id_of_id]]);
        }
        bool multithread = (ParameterServer::instance()->get<bool>("concurrent_edge_construction"));
        if(multithread)
        {
        	printMultiThreadInfo("node comparison for rgbdicp");
			results = QtConcurrent::blockingMapped(
					nodes_to_comp, boost::bind(&Node::matchNodePair, graph_[i], _1, 20));	//ROSS-TODO:: 30 a parameter
        }
        else
        {
			for(size_t k = 0; k < nodes_to_comp.size(); k++)
			{
				results.push_back(graph_[i]->matchNodePair(nodes_to_comp[k], 20));
			}
        }

		std::ofstream myfile;
		std::stringstream filename;
        QList<MatchingResult> rgbdicpList;
        for (uint j = 0; j < results.size(); j++) {
			MatchingResult& mr = results[j];
			if(mr.edge.id1 < 0)		// bad ransac
					continue;

			mr.icp_trafo = getGraphTransformBetweenNodes(mr.edge.id2, mr.edge.id1);
			if (checkEigenMatrixhasNaNs(mr.icp_trafo))
			{
				ROS_ERROR_STREAM("transformation from graph has NaNs. Trafo:\n" << mr.icp_trafo);
				continue;
			}
			rgbdicpList.push_back(mr);

			pointcloud_type tempConverged;
			filename.str("");
			filename << "node_" << mr.edge.id2 << "_graph_transformed_to_node_" << mr.edge.id1 << ".txt";
		    myfile.open (filename.str().c_str());
		    myfile << mr.icp_trafo;
		    myfile.close();
//		    filename << ".pcd";
//			transformPointCloud (*(graph_[mr.edge.id2]->pc_col), tempConverged,  mr.icp_trafo);
//			writer.write (filename.str(), tempConverged, true);
        }

        // this is how to multithread the joint optimization (not stable)
//        bool icpMultithread = false;
//        if(!icpMultithread)
//        {
//        	for(uint j=0; j< rgbdicpList.size(); j++)
//        	{
//        		performJointOptimization(rgbdicpList[j]);
//        	}
//        }
//		else
//		{
//			printMultiThreadInfo("joint optimization");
//			QtConcurrent::blockingMap(rgbdicpList, boost::bind(&GraphManager::performJointOptimization, this, _1));
//		}

        for(uint j=0; j < rgbdicpList.size(); j++)
        {
        	MatchingResult & mr = rgbdicpList[j];

    		performJointOptimization(mr);
			filename.str("");
			filename << "node_" << graph_[i]->id_ << "_converged_to_node_" << mr.edge.id1 << ".txt";
			myfile.open (filename.str().c_str());
			myfile << mr.final_trafo;
			myfile.close();
//			transformPointCloud (*(graph_[i]->pc_col), tempConverged,  mr.final_trafo);
//			writer.write (filename.str(), tempConverged, true);

			// source = current node, target = other node. Transform is from source to target
			// mr.edge.id2 = source mr.edge.id1 = target
			ROS_INFO_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << "\n" << mr.edge.informationMatrix);

			if (addEdgeToG2O(mr.edge, isBigTrafo(mr.edge.mean),
					mr.inlier_matches.size() > last_inlier_matches_.size())) { //TODO: result isBigTrafo is not considered
				ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
				if (mr.inlier_matches.size() > last_inlier_matches_.size()) {
					last_matching_node_ = mr.edge.id1;
					last_inlier_matches_ = mr.inlier_matches;
					last_matches_ = mr.all_matches;
				}
			}
        }
    }
    optimizeGraph();
}

void GraphManager::performJointOptimization(MatchingResult& mr)
{
	uint sourceId = mr.edge.id2;
	uint targetId = mr.edge.id1;
    // RGBD ICP
    ROS_INFO_STREAM("Performing RGBDICP with source node(" << sourceId << ") and target node (" << targetId << ")");

    pcl::IterativeClosestPoint<PointNormal, PointNormal> icp;
    // set source and target clouds from indices of pointclouds
	pcl::ExtractIndices<PointNormal> handlefilter;
	pcl::PointIndices::Ptr sourceHandleIndices (new pcl::PointIndices);
	pointcloud_type::Ptr cloudHandlesSource (new pointcloud_type);
	sourceHandleIndices->indices = graph_[sourceId]->handleIndices;
	handlefilter.setIndices(sourceHandleIndices);
	handlefilter.setInputCloud(graph_[sourceId]->pc_col);
	handlefilter.filter(*cloudHandlesSource);
	icp.setInputCloud(cloudHandlesSource);

	pcl::PointIndices::Ptr targetHandleIndices (new pcl::PointIndices);
	pointcloud_type::Ptr cloudHandlesTarget (new pointcloud_type);
	targetHandleIndices->indices = graph_[targetId]->handleIndices;
	handlefilter.setIndices(targetHandleIndices);
	handlefilter.setInputCloud(graph_[targetId]->pc_col);
	handlefilter.filter(*cloudHandlesTarget);
	icp.setInputTarget(cloudHandlesTarget);

	PointCloudNormal Final;
	icp.align(Final, mr.icp_trafo);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	std::ofstream myfile;
	std::stringstream filename;
	pointcloud_type tempConverged;
	filename << "node_" << sourceId << "_handle_transformed_to_node_" << targetId << ".txt";
	myfile.open (filename.str().c_str());
	myfile << icp.getFinalTransformation();
	myfile.close();
//	transformPointCloud (*(this->pc_col), tempConverged,  icp.getFinalTransformation());
//	writer.write (filename.str(), tempConverged, true);

	ROS_INFO("Initialize transformation estimation object....");
	boost::shared_ptr< TransformationEstimationJointOptimize<PointNormal, PointNormal > >
		transformationEstimation_(new TransformationEstimationJointOptimize<PointNormal, PointNormal>());

	float denseCloudWeight = 1.0;
	float visualFeatureWeight = 0.0;
	float handleFeatureWeight = 0.25;
	transformationEstimation_->setWeights(denseCloudWeight, visualFeatureWeight, handleFeatureWeight);

	std::vector<int> sourceSIFTIndices, targetSIFTIndices;
	graph_[sourceId]->getFeatureIndices(graph_[targetId],mr,sourceSIFTIndices,targetSIFTIndices);
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
	icpJointOptimize.setInputCloud(graph_[sourceId]->pc_col);
	icpJointOptimize.setInputTarget(graph_[targetId]->pc_col);

	ROS_INFO("Running ICP....");
	PointCloudNormal::Ptr cloud_transformed( new PointCloudNormal);
	icpJointOptimize.align ( *cloud_transformed, icp.getFinalTransformation()); //init_tr );
	std::cout << "[SIIMCloudMatch::runICPMatch] Has converged? = " << icpJointOptimize.hasConverged() << std::endl <<
				"	fitness score (SSD): " << icpJointOptimize.getFitnessScore (1000) << std::endl
				<<	icpJointOptimize.getFinalTransformation () << "\n";

	mr.final_trafo = icpJointOptimize.getFinalTransformation();
    mr.edge.mean = eigen2G2O(mr.final_trafo.cast<double>());//we insert an edge between the frames
    double w = 80;
    mr.edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity()*(w*w); //TODO: What
}

void GraphManager::visualizeGraphEdges() const {
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

    if (marker_pub_.getNumSubscribers() > 0){ //no visualization for nobody
        visualization_msgs::Marker edges_marker;
        edges_marker.header.frame_id = "/openni_rgb_optical_frame"; //TODO: Should be a meaningfull fixed frame with known relative pose to the camera
        edges_marker.header.stamp = ros::Time::now();
        edges_marker.ns = "camera_pose_graph"; // Set the namespace and id for this marker.  This serves to create a unique ID
        edges_marker.id = 0;    // Any marker sent with the same namespace and id will overwrite the old one

        edges_marker.type = visualization_msgs::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::Marker::ADD; // Set the marker action.  Options are ADD and DELETE
        edges_marker.frame_locked = true; //rviz automatically retransforms the markers into the frame every update cycle
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        edges_marker.scale.x = 0.005; //line width
        //Global pose (used to transform all points)
        edges_marker.pose.position.x = 0;
        edges_marker.pose.position.y = 0;
        edges_marker.pose.position.z = 0;
        edges_marker.pose.orientation.x = 0.0;
        edges_marker.pose.orientation.y = 0.0;
        edges_marker.pose.orientation.z = 0.0;
        edges_marker.pose.orientation.w = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        edges_marker.color.r = 1.0f;
        edges_marker.color.g = 1.0f;
        edges_marker.color.b = 1.0f;
        edges_marker.color.a = 0.5f;//looks smoother
        geometry_msgs::Point point; //start and endpoint for each line segment
        g2o::VertexSE3* v1,* v2; //used in loop
        EdgeSet::iterator edge_iter = optimizer_->edges().begin();
        int counter = 0;
        for(;edge_iter != optimizer_->edges().end(); edge_iter++, counter++) {
            g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
            std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
            v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
            v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));

            point.x = v1->estimate().translation().x();
            point.y = v1->estimate().translation().y();
            point.z = v1->estimate().translation().z();
            edges_marker.points.push_back(point);
            
            point.x = v2->estimate().translation().x();
            point.y = v2->estimate().translation().y();
            point.z = v2->estimate().translation().z();
            edges_marker.points.push_back(point);
        }

        marker_pub_.publish (edges_marker);
        ROS_INFO("published %d graph edges", counter);
    }

    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GraphManager::visualizeGraphNodes() const {
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

    if (marker_pub_.getNumSubscribers() > 0){ //don't visualize, if nobody's looking
        visualization_msgs::Marker nodes_marker;
        nodes_marker.header.frame_id = "/openni_rgb_optical_frame"; //TODO: Should be a meaningfull fixed frame with known relative pose to the camera
        nodes_marker.header.stamp = ros::Time::now();
        nodes_marker.ns = "camera_pose_graph"; // Set the namespace and id for this marker.  This serves to create a unique ID
        nodes_marker.id = 1;    // Any marker sent with the same namespace and id will overwrite the old one


        nodes_marker.type = visualization_msgs::Marker::LINE_LIST;
        nodes_marker.action = visualization_msgs::Marker::ADD; // Set the marker action.  Options are ADD and DELETE
        nodes_marker.frame_locked = true; //rviz automatically retransforms the markers into the frame every update cycle
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        nodes_marker.scale.x = 0.002;
        //Global pose (used to transform all points) //TODO: is this the default pose anyway?
        nodes_marker.pose.position.x = 0;
        nodes_marker.pose.position.y = 0;
        nodes_marker.pose.position.z = 0;
        nodes_marker.pose.orientation.x = 0.0;
        nodes_marker.pose.orientation.y = 0.0;
        nodes_marker.pose.orientation.z = 0.0;
        nodes_marker.pose.orientation.w = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        nodes_marker.color.r = 1.0f;
        nodes_marker.color.g = 0.0f;
        nodes_marker.color.b = 0.0f;
        nodes_marker.color.a = 1.0f;


        geometry_msgs::Point tail; //same startpoint for each line segment
        geometry_msgs::Point tip;  //different endpoint for each line segment
        std_msgs::ColorRGBA arrow_color_red  ;  //red x axis
        arrow_color_red.r = 1.0;
        arrow_color_red.a = 1.0;
        std_msgs::ColorRGBA arrow_color_green;  //green y axis
        arrow_color_green.g = 1.0;
        arrow_color_green.a = 1.0;
        std_msgs::ColorRGBA arrow_color_blue ;  //blue z axis
        arrow_color_blue.b = 1.0;
        arrow_color_blue.a = 1.0;
        Eigen::Vector3d origin(0.0,0.0,0.0);
        Eigen::Vector3d x_axis(0.2,0.0,0.0); //20cm long axis for the first (almost fixed) node
        Eigen::Vector3d y_axis(0.0,0.2,0.0);
        Eigen::Vector3d z_axis(0.0,0.0,0.2);
        Eigen::Vector3d tmp; //the transformed endpoints
        int counter = 0;
        g2o::VertexSE3* v; //used in loop
        VertexIDMap::iterator vertex_iter = optimizer_->vertices().begin();
        for(/*see above*/; vertex_iter != optimizer_->vertices().end(); vertex_iter++, counter++) {
            v = dynamic_cast<g2o::VertexSE3* >((*vertex_iter).second);
            //v->estimate().rotation().x()+ v->estimate().rotation().y()+ v->estimate().rotation().z()+ v->estimate().rotation().w();
            tmp = v->estimate() * origin;
            tail.x = tmp.x();
            tail.y = tmp.y();
            tail.z = tmp.z();
            //Endpoints X-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_red);
            tmp = v->estimate() * x_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_red);
            //Endpoints Y-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_green);
            tmp = v->estimate() * y_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_green);
            //Endpoints Z-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_blue);
            tmp = v->estimate() * z_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_blue);
            //shorten all nodes after the first one
            x_axis.x() = 0.1;
            y_axis.y() = 0.1;
            z_axis.z() = 0.1;
        }

        marker_pub_.publish (nodes_marker);
        ROS_INFO("published %d graph nodes", counter);
    }

    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

bool GraphManager::addEdgeToG2O(const LoadedEdge3D& edge, bool largeEdge, bool set_estimate) {
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

    QMutexLocker locker(&optimizer_mutex);
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(edge.id1));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(edge.id2));

    ROS_INFO_STREAM("edge mean:" << edge.mean);

    // at least one vertex has to be created, assert that the transformation
    // is large enough to avoid to many vertices on the same spot
    if (!v1 || !v2){
        if (!largeEdge) {
            ROS_INFO("Edge to new vertex is to short, vertex will not be inserted");
            return false; 
        }
    }

    if(!v1 && !v2){
      ROS_ERROR("Missing both vertices: %i, %i, cannot create edge", edge.id1, edge.id2);
      return false;
    }
    else if (!v1 && v2) {
        v1 = new g2o::VertexSE3;
        assert(v1);
        v1->setId(edge.id1);
        v1->setEstimate(v2->estimate() * edge.mean.inverse());
        optimizer_->addVertex(v1); 
        latest_transform_ = g2o2QMatrix(v1->estimate()); 
    }
    else if (!v2 && v1) {
        v2 = new g2o::VertexSE3;
        assert(v2);
        v2->setId(edge.id2);
        v2->setEstimate(v1->estimate() * edge.mean);
        optimizer_->addVertex(v2); 
        latest_transform_ = g2o2QMatrix(v2->estimate()); 
    }
    else if(set_estimate){
        v2->setEstimate(v1->estimate() * edge.mean);
    }
    g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3;
    g2o_edge->vertices()[0] = v1;
    g2o_edge->vertices()[1] = v2;
    g2o_edge->setMeasurement(edge.mean);
    g2o_edge->setInverseMeasurement(edge.mean.inverse());
    g2o_edge->setInformation(edge.informationMatrix);
    optimizer_->addEdge(g2o_edge);

    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    return true;
}

void GraphManager::optimizeGraph(int iter){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    int iterations = iter >= 0 ? iter : ParameterServer::instance()->get<int>("optimizer_iterations");
    QMutexLocker locker(&optimizer_mutex);

    ROS_WARN("Starting Optimization");
    std::string bagfile_name = ParameterServer::instance()->get<std::string>("bagfile_name");
    optimizer_->save((bagfile_name + "_g2o-optimizer-save-file-before").c_str());
    optimizer_->initializeOptimization();
    for(int i = 0; i <  iterations; i++){
      int currentIt = optimizer_->optimize(1);
      optimizer_->computeActiveErrors();
      ROS_INFO_STREAM("G2O Statistics: " << optimizer_->vertices().size() << " nodes, " 
                      << optimizer_->edges().size() << " edges. "
                      << "chi2: " << optimizer_->chi2() << ", Iterations: " << currentIt);
    }
    optimizer_->save((bagfile_name + "_g2o-optimizer-save-file-after").c_str());


    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(optimizer_->vertices().size()-1));

    computed_motion_ =  g2o2TF(v->estimate());
    latest_transform_ = g2o2QMatrix(v->estimate()); 
    Q_EMIT setGraphEdges(getGraphEdges());
    Q_EMIT updateTransforms(getAllPosesAsMatrixList());

    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GraphManager::broadcastTransform(const ros::TimerEvent& ){
    ros::Time now = ros::Time::now();
    std::string fixed_frame = ParameterServer::instance()->get<std::string>("fixed_frame_name");
    std::string base_frame  = ParameterServer::instance()->get<std::string>("base_frame_name");
    
    if(graph_.size() > 0 && !batch_processing_runs_ ) {
        Node* node = graph_[graph_.size()-1]; //latest node
        base2points_ = node->getBase2PointsTransform();

        tf::Transform  world2base;
        world2base = init_base_pose_*base2points_*computed_motion_*base2points_.inverse();
        br_.sendTransform(tf::StampedTransform(world2base, now, fixed_frame, base_frame));

        //visualize the transformation
        std::stringstream ss;
        Eigen::Matrix4f transform;
        pcl_ros::transformAsMatrix(computed_motion_, transform);
        ss << "<b>Current Camera Transformation w.r.t. the Initial Frame</b>";
        ss << "<pre>" <<  transform << "</pre>";
        QString mystring(ss.str().c_str());
        Q_EMIT newTransformationMatrix(mystring);
    } 
    else //no graph, no map, send initial pose (usually identity)
    {
        br_.sendTransform(tf::StampedTransform(init_base_pose_, now, fixed_frame, base_frame));
    }

}

/**
 * Publish the updated transforms for the graph node resp. clouds
 *
void GraphManager::publishCorrectedTransforms(){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    //fill message
    rgbdslam::CloudTransforms msg;
    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        g2o::VertexSE3* v = optimizer_->vertex(i);
        tf::Transform trans = g2o2TF(v->estimate());
        geometry_msgs::Transform trans_msg;
        tf::transformTFToMsg(trans,trans_msg);
        msg.transforms.push_back(trans_msg);
        msg.ids.push_back(graph_[i]->msg_id_); //msg_id is no more
    }
    msg.header.stamp = ros::Time::now();

    if (transform_pub_.getNumSubscribers() > 0)
        transform_pub_.publish(msg);
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}*/

void GraphManager::reset(){
    reset_request_ = true;
}

void GraphManager::deleteLastFrame(){
    if(graph_.size() <= 1) {
      ROS_INFO("Resetting, as the only node is to be deleted");
      reset_request_ = true;
      Q_EMIT deleteLastNode();
      return;
    }
    optimizer_mutex.lock();
    g2o::VertexSE3* v_to_del = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(optimizer_->vertices().size()-1));//last vertex
    g2o::VertexSE3 *v1, *v2; //used in loop as temporaries
    EdgeSet::iterator edge_iter = optimizer_->edges().begin();
    for(;edge_iter != optimizer_->edges().end(); edge_iter++) {
        g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
        std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
        v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
        v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));
        if(v1->id() == v_to_del->id() || v2->id() == v_to_del->id()) 
          optimizer_->removeEdge((*edge_iter));
    }

    optimizer_->removeVertex(v_to_del);
    graph_.erase(graph_.size()-1);
    optimizer_mutex.unlock();
    Q_EMIT deleteLastNode();
    optimizeGraph();//s.t. the effect of the removed edge transforms are removed to
    ROS_INFO("Removed most recent node");
    Q_EMIT setGUIInfo("Removed most recent node");
    //Q_EMIT setGraphEdges(getGraphEdges());
    //updateTransforms needs to be last, as it triggers a redraw
    //Q_EMIT updateTransforms(getAllPosesAsMatrixList());
}

QList<QPair<int, int> >* GraphManager::getGraphEdges()
{
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    //QList<QPair<int, int> >* edge_list = new QList<QPair<int, int> >();
    current_edges_.clear();
    g2o::VertexSE3 *v1, *v2; //used in loop
    EdgeSet::iterator edge_iter = optimizer_->edges().begin();
    for(;edge_iter != optimizer_->edges().end(); edge_iter++) {
        g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
        std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
        v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
        v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));
        current_edges_.append( qMakePair(v1->id(), v2->id()));
    }
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    return new QList<QPair<int, int> >(current_edges_);
}

QList<QMatrix4x4>* GraphManager::getAllPosesAsMatrixList(){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    ROS_DEBUG("Retrieving all transformations from optimizer");
    //QList<QMatrix4x4>* result = new QList<QMatrix4x4>();
    current_poses_.clear();
#if defined(QT_VERSION) && QT_VERSION >= 0x040700
    current_poses_.reserve(optimizer_->vertices().size());//only allocates the internal pointer array
#endif

    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
        if(v){ 
            current_poses_.push_back(g2o2QMatrix(v->estimate())); 
        } else {
            ROS_ERROR("Nullpointer in graph at position %i!", i);
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    return new QList<QMatrix4x4>(current_poses_); //pointer to a copy
}

// If QT Concurrent is available, run the saving in a seperate thread
void GraphManager::saveAllClouds(QString filename, bool threaded){
    if (ParameterServer::instance()->get<bool>("concurrent_edge_construction") && threaded) {
        QFuture<void> f1 = QtConcurrent::run(this, &GraphManager::saveAllCloudsToFile, filename);
        //f1.waitForFinished();
    }
    else {// Otherwise just call it without threading
        saveAllCloudsToFile(filename);
    }
}

void GraphManager::saveIndividualClouds(QString filename, bool threaded){
  if (ParameterServer::instance()->get<bool>("concurrent_edge_construction") && threaded) {
    QFuture<void> f1 = QtConcurrent::run(this, &GraphManager::saveIndividualCloudsToFile, filename);
    //f1.waitForFinished();
  }
  else {
    saveIndividualCloudsToFile(filename);
  }
}

void GraphManager::saveIndividualCloudsToFile(QString file_basename)
{
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    ROS_INFO("Saving all clouds to %sxxxx.pcd", qPrintable(file_basename));
    std::string gt = ParameterServer::instance()->get<std::string>("ground_truth_frame_name");
    ROS_INFO_COND(!gt.empty(), "Saving all clouds with ground truth sensor position to gt_%sxxxx.pcd", qPrintable(file_basename));

    batch_processing_runs_ = true;
    tf::Transform  world2base;
    QString message, filename;
    std::string fixed_frame_id = ParameterServer::instance()->get<std::string>("fixed_frame_name");
    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
        if(!v){ 
            ROS_ERROR("Nullpointer in graph at position %i!", i);
            continue;
        }
        if(graph_[i]->pc_col->size() == 0){
            ROS_INFO("Skipping Node %i, point cloud data is empty!", i);
            continue;
        }
        /*/TODO: is all this correct?
        tf::Transform transform = g2o2TF(v->estimate());
        tf::Transform cam2rgb;
        cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
        cam2rgb.setOrigin(tf::Point(0,-0.04,0));
        world2base = cam2rgb*transform;
        */
        tf::Transform pose = g2o2TF(v->estimate());
        tf::StampedTransform base2points = graph_[i]->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time
        world2base = init_base_pose_*base2points*pose*base2points.inverse();

        Eigen::Vector4f sensor_origin(world2base.getOrigin().x(),world2base.getOrigin().y(),world2base.getOrigin().z(),world2base.getOrigin().w());
        Eigen::Quaternionf sensor_orientation(world2base.getRotation().w(),world2base.getRotation().x(),world2base.getRotation().y(),world2base.getRotation().z());

        graph_[i]->pc_col->sensor_origin_ = sensor_origin;
        graph_[i]->pc_col->sensor_orientation_ = sensor_orientation;
        graph_[i]->pc_col->header.frame_id = fixed_frame_id;

        filename.sprintf("%s_%04d.pcd", qPrintable(file_basename), i);
        Q_EMIT setGUIStatus(message.sprintf("Saving to %s: Transformed Node %i/%i", qPrintable(filename), i, (int)optimizer_->vertices().size()));
        pcl::io::savePCDFile(qPrintable(filename), *(graph_[i]->pc_col), true); //Last arg: true is binary mode. ASCII mode drops color bits
        
        if(!gt.empty()){
          tf::StampedTransform gt_world2base = graph_[i  ]->getGroundTruthTransform();//get mocap pose of base in map
          if( gt_world2base.frame_id_   == "/missing_ground_truth" ){ 
            ROS_WARN_STREAM("Skipping ground truth: " << gt_world2base.child_frame_id_ << " child/parent " << gt_world2base.frame_id_);
            continue;
          }
          Eigen::Vector4f sensor_origin(gt_world2base.getOrigin().x(),gt_world2base.getOrigin().y(),gt_world2base.getOrigin().z(),gt_world2base.getOrigin().w());
          Eigen::Quaternionf sensor_orientation(gt_world2base.getRotation().w(),gt_world2base.getRotation().x(),gt_world2base.getRotation().y(),gt_world2base.getRotation().z());

          graph_[i]->pc_col->sensor_origin_ = sensor_origin;
          graph_[i]->pc_col->sensor_orientation_ = sensor_orientation;
          graph_[i]->pc_col->header.frame_id = fixed_frame_id;

          filename.sprintf("%s_%04d_gt.pcd", qPrintable(file_basename), i);
          Q_EMIT setGUIStatus(message.sprintf("Saving to %s: Transformed Node %i/%i", qPrintable(filename), i, (int)optimizer_->vertices().size()));
          pcl::io::savePCDFile(qPrintable(filename), *(graph_[i]->pc_col), true); //Last arg: true is binary mode. ASCII mode drops color bits
        }
  
    }
    Q_EMIT setGUIStatus("Saved all point clouds");
    ROS_INFO ("Saved all points clouds to %sxxxx.pcd", qPrintable(file_basename));
    batch_processing_runs_ = false;
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GraphManager::saveAllCloudsToFile(QString filename){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    pointcloud_type aggregate_cloud; ///will hold all other clouds
    pointcloud_type::Ptr nodeCloud (new pointcloud_type);
    pointcloud_type::Ptr nodeCloudFiltered (new pointcloud_type);
    ROS_INFO("Saving all clouds to %s, this may take a while as they need to be transformed to a common coordinate frame.", qPrintable(filename));
    batch_processing_runs_ = true;
    tf::Transform  world2cam;
    //fill message
    //rgbdslam::CloudTransforms msg;
    QString message;
    tf::Transform cam2rgb;
    cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    cam2rgb.setOrigin(tf::Point(0,-0.04,0));
    uint pointcloud_skip = ParameterServer::instance()->get<int>("pointcloud_skip_step");
    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
    	if(i%pointcloud_skip != 0)
    		continue;	// for high overlapping recordings
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
        if(!v){ 
            ROS_ERROR("Nullpointer in graph at position %i!", i);
            continue;
        }

        tf::Transform transform = g2o2TF(v->estimate());
        world2cam = cam2rgb*transform;
        //recall cached clouds on the hdd
        std::stringstream nodeName;
        nodeName << "node_" << i << ".pcd";
        ROS_INFO_STREAM("Retrieving node " << nodeName.str() << " from cache and saving to map");
        pcl::PCDReader nodeFetcher;
        nodeFetcher.read(nodeName.str(), *nodeCloud);
        pcl::VoxelGrid<point_type> sor;
        sor.setInputCloud (nodeCloud);
        sor.setLeafSize (0.001f, 0.001f, 0.001f);
        sor.filter (*nodeCloudFiltered);
        transformAndAppendPointCloud (*nodeCloudFiltered, aggregate_cloud, world2cam, Max_Depth);
        nodeCloud->clear();
        nodeCloudFiltered->clear();
        //transformAndAppendPointCloud (*(graph_[i]->pc_col), aggregate_cloud, world2cam, Max_Depth);

        if(ParameterServer::instance()->get<bool>("batch_processing"))
          graph_[i]->clearPointCloud(); //saving all is the last thing to do, so these are not required anymore
        Q_EMIT setGUIStatus(message.sprintf("Saving to %s: Transformed Node %i/%i", qPrintable(filename), i, (int)optimizer_->vertices().size()));
    }
    aggregate_cloud.header.frame_id = "/openni_camera";
    if(filename.endsWith(".ply", Qt::CaseInsensitive))
      pointCloud2MeshFile(filename, aggregate_cloud);
    if(filename.endsWith(".pcd", Qt::CaseInsensitive))
      pcl::io::savePCDFile(qPrintable(filename), aggregate_cloud, true); //Last arg is binary mode
    else {
      ROS_WARN("Filename misses correct extension (.pcd or .ply) using .pcd");
      filename.append(".pcd");
      pcl::io::savePCDFile(qPrintable(filename), aggregate_cloud, true); //Last arg is binary mode
    }
    Q_EMIT setGUIStatus(message.sprintf("Saved %d data points to %s", (int)aggregate_cloud.points.size(), qPrintable(filename)));
    ROS_INFO ("Saved %d data points to %s", (int)aggregate_cloud.points.size(), qPrintable(filename));

    if (whole_cloud_pub_.getNumSubscribers() > 0){ //if it should also be send out
        sensor_msgs::PointCloud2 cloudMessage_; //this will be send out in batch mode
        pcl::toROSMsg(aggregate_cloud,cloudMessage_);
        cloudMessage_.header.frame_id = "/openni_camera";
        cloudMessage_.header.stamp = ros::Time::now();
        whole_cloud_pub_.publish(cloudMessage_);
        ROS_INFO("Aggregate pointcloud sent");
    }
    batch_processing_runs_ = false;
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GraphManager::pointCloud2MeshFile(QString filename, pointcloud_type full_cloud){
  QFile file(filename);//file is closed on destruction
  if(!file.open(QIODevice::WriteOnly|QIODevice::Text)) return; //TODO: Errormessage
  QTextStream out(&file);
	out << "ply\n";
	out << "format ascii 1.0\n";
	out << "element vertex " << (int)full_cloud.points.size() << "\n"; 
	out << "property float x\n";
	out << "property float y\n";
	out << "property float z\n";
	out << "property uchar red\n";
	out << "property uchar green\n";
	out << "property uchar blue\n";
	out << "end_header\n";
  unsigned char r,g,b;
  float x, y, z ;
  for(unsigned int i = 0; i < full_cloud.points.size() ; i++){
    b = *(  (unsigned char*)&(full_cloud.points[i].rgb));
    g = *(1+(unsigned char*)&(full_cloud.points[i].rgb));
    r = *(2+(unsigned char*)&(full_cloud.points[i].rgb));
    x = full_cloud.points[i].x;
    y = full_cloud.points[i].y;
    z = full_cloud.points[i].z;
    out << qSetFieldWidth(8) << x << " " << y << " " << z << " ";
    out << qSetFieldWidth(3) << r << " " << g << " " << b << "\n";
  }
}
  

void GraphManager::saveTrajectory(QString filebasename){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    if(graph_.size() == 0){
      ROS_ERROR("Graph is empty, no trajectory can be saved");
      return;
    }
    ROS_INFO("Logging Trajectory");
    QMutexLocker locker(&optimizer_mutex);
    std::string gt = ParameterServer::instance()->get<std::string>("ground_truth_frame_name");

    ROS_INFO("Comparison of relative motion with ground truth");
    QString gtt_fname("_ground_truth.txt");
    QFile gtt_file(gtt_fname.prepend(filebasename));//file is closed on destruction
    if(!gtt_file.open(QIODevice::WriteOnly|QIODevice::Text)) return; //TODO: Errormessage
    QTextStream gtt_out(&gtt_file);
    tf::StampedTransform b2p = graph_[0]->getGroundTruthTransform();
    gtt_out.setRealNumberNotation(QTextStream::FixedNotation);
    gtt_out << "# TF Coordinate Frame ID: " << b2p.frame_id_.c_str() << "(data: " << b2p.child_frame_id_.c_str() << ")\n";

     
    QString et_fname("_estimate.txt");
    QFile et_file (et_fname.prepend(filebasename));//file is closed on destruction
    if(!et_file.open(QIODevice::WriteOnly|QIODevice::Text)) return; //TODO: Errormessage
    QTextStream et_out(&et_file);
    et_out.setRealNumberNotation(QTextStream::FixedNotation);
    b2p = graph_[0]->getBase2PointsTransform();
    et_out << "# TF Coordinate Frame ID: " << b2p.frame_id_.c_str() << "(data: " << b2p.child_frame_id_.c_str() << ")\n";

    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
        ROS_ERROR_COND(!v, "Nullpointer in graph at position %i!", i);

        tf::Transform pose = g2o2TF(v->estimate());

        tf::StampedTransform base2points = graph_[i]->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time
        tf::Transform world2base = init_base_pose_*base2points*pose*base2points.inverse();

        logTransform(et_out, world2base, graph_[i]->pc_col->header.stamp.toSec()); 
        //Eigen::Matrix<double, 6,6> uncertainty = v->uncertainty();
        //et_out << uncertainty(0,0) << "\t" << uncertainty(1,1) << "\t" << uncertainty(2,2) << "\t" << uncertainty(3,3) << "\t" << uncertainty(4,4) << "\t" << uncertainty(5,5) <<"\n" ;
        if(!gt.empty()){
          tf::StampedTransform gt_world2base = graph_[i  ]->getGroundTruthTransform();//get mocap pose of base in map
          if( gt_world2base.frame_id_   == "/missing_ground_truth" ){ 
            ROS_WARN_STREAM("Skipping ground truth: " << gt_world2base.child_frame_id_ << " child/parent " << gt_world2base.frame_id_);
            continue;
          }
          logTransform(gtt_out, gt_world2base, gt_world2base.stamp_.toSec()); 
          //logTransform(et_out, world2base, gt_world2base.stamp_.toSec()); 
        } 
    }
    ROS_INFO_COND(!gt.empty(), "Written logfiles ground_truth_trajectory.txt and estimated_trajectory.txt");
    ROS_INFO_COND(gt.empty(),  "Written logfile estimated_trajectory.txt");
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}


void GraphManager::sendAllClouds(){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    /*
    if (batch_cloud_pub_.getNumSubscribers() == 0){
        ROS_WARN("No Subscribers: Sending of clouds cancelled");
        return;
    }*/

    PointCloudCapturer dataCapturer;

    ROS_INFO("Sending out all clouds");
    batch_processing_runs_ = true;
    ros::Rate r(5); //slow down a bit, to allow for transmitting to and processing in other nodes
    uint pointcloud_skip = ParameterServer::instance()->get<int>("pointcloud_skip_step");

    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
    	if(i%pointcloud_skip != 0)
    	    continue;	// for high overlapping recordings
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
        if(!v){ 
            ROS_ERROR("Nullpointer in graph at position %i!", i);
            continue;
        }

        tf::Transform base2points = graph_[i]->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time
        printTransform("base2points", base2points);
        tf::Transform computed_motion = g2o2TF(v->estimate());//get pose of point cloud w.r.t. first frame's pc
        printTransform("computed_motion", computed_motion);
        printTransform("init_base_pose_", init_base_pose_);

        tf::Transform world2base = init_base_pose_*base2points*computed_motion*base2points.inverse();
        tf::Transform gt_world2base = graph_[i]->getGroundTruthTransform();//get mocap pose of base in map
        tf::Transform err = gt_world2base.inverseTimes(world2base);
        //TODO: Compute err from relative transformations betw. time steps

        ros::Time now = ros::Time::now(); //makes sure things have a corresponding timestamp
        ROS_DEBUG("Sending out transform %i", i);
        printTransform("World->Base", world2base);
        std::string fixed_frame = ParameterServer::instance()->get<std::string>("fixed_frame_name");
        br_.sendTransform(tf::StampedTransform(world2base, now, fixed_frame, "/openni_camera"));
        br_.sendTransform(tf::StampedTransform(err, now, fixed_frame, "/where_mocap_should_be"));

        ROS_DEBUG("Sending out cloud %i", i);
        //graph_[i]->publish("/batch_transform", now, batch_cloud_pub_);
        graph_[i]->publish("/openni_rgb_optical_frame", now, batch_cloud_pub_);
        //tf::Transform ground_truth_tf = graph_[i]->getGroundTruthTransform();
        QString message;
        Q_EMIT setGUIInfo(message.sprintf("Sending pointcloud and map transform (%i/%i) on topics %s and /tf", (int)i+1, (int)optimizer_->vertices().size(), ParameterServer::instance()->get<std::string>("individual_cloud_out_topic").c_str()) );

        dataCapturer.saveCloudsToBagfile(graph_[i], world2base);

        r.sleep();
    }

    batch_processing_runs_ = false;
    Q_EMIT sendFinished();
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GraphManager::setMaxDepth(float max_depth){
	Max_Depth = max_depth;
	ROS_INFO("Max Depth set to: %f", max_depth);
}

void GraphManager::cloudRendered(pointcloud_type const * pc) {
  for(int i = graph_.size()-1; i >= 0;  i--){
    if(graph_[i]->pc_col.get() == pc){
      graph_[i]->clearPointCloud();
      ROS_WARN("Cleared PointCloud after rendering to openGL list. It will not be available for save/send.");
      return;
    }
  }
}


void GraphManager::sanityCheck(float thresh){ 
  thresh *=thresh; //squaredNorm
  QMutexLocker locker(&optimizer_mutex);
  EdgeSet::iterator edge_iter = optimizer_->edges().begin();
  for(int i =0;edge_iter != optimizer_->edges().end(); edge_iter++, i++) {
      g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
      Eigen::Vector3d ev = myedge->measurement().translation();
      if(ev.squaredNorm() > thresh){
        optimizer_->removeEdge(myedge); 
      }
  }
}
void GraphManager::pruneEdgesWithErrorAbove(float thresh){
  QMutexLocker locker(&optimizer_mutex);
  EdgeSet::iterator edge_iter = optimizer_->edges().begin();
  for(int i =0;edge_iter != optimizer_->edges().end(); edge_iter++, i++) {
      g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
      g2o::EdgeSE3::ErrorVector ev = myedge->error();
      if(ev.squaredNorm() > thresh){
        optimizer_->removeEdge(myedge); 
      }
  }
}
void GraphManager::printEdgeErrors(QString filename){
  QMutexLocker locker(&optimizer_mutex);
  std::fstream filestr;
  filestr.open (qPrintable(filename),  std::fstream::out );

  EdgeSet::iterator edge_iter = optimizer_->edges().begin();
  for(int i =0;edge_iter != optimizer_->edges().end(); edge_iter++, i++) {
      g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
      g2o::EdgeSE3::ErrorVector ev = myedge->error();
      ROS_INFO_STREAM("Error Norm for edge " << i << ": " << ev.squaredNorm());
      filestr << "Error for edge " << i << ": " << ev.squaredNorm() << std::endl;
  }
  filestr.close();
}

bool GraphManager::isBusy(){
  return (batch_processing_runs_ || process_node_runs_ );
}

double GraphManager::geodesicDiscount(g2o::HyperDijkstra& hypdij, const MatchingResult& mr){
    //Discount by geodesic distance to root node
    const g2o::HyperDijkstra::AdjacencyMap am = hypdij.adjacencyMap();
    g2o::VertexSE3* older_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(mr.edge.id1));
    double discount_factor = am.at(older_vertex).distance();
    discount_factor = discount_factor > 0.0? 1.0/discount_factor : 1.0;//avoid inf
    ROS_INFO("Discount weight for connection to Node %i = %f", mr.edge.id1, discount_factor);
    return discount_factor;
}
