/*
 * rgbd_icp.h
 *
 *  Created on: Feb 13, 2012
 *      Author: ross
 */

#ifndef RGBD_ICP_H_
#define RGBD_ICP_H_



//local
#include "graphnode.h"
#include "edge.h"

//rgbdslam
#include "rgbdslam/featureMatch.h"
#include "rgbdslam/match.h"

//vector/eigen
#include <vector>
#include "eigen_conversions/eigen_msg.h"

//opencv
#include <opencv2/features2d/features2d.hpp>

//pcl
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>

//g2o
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/math_groups/se3quat.h"
#include "g2o/types/slam3d/edge_se3_quat.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

class rgbd_icp
{
public:

	// Constructor
	rgbd_icp();

	// This calls computeRGBD_ICP, as well as adding the node to the graph, run optimizer etc.
	void processRGBD_ICP(const rgbdslam::featureMatch& msg);

	// This calcuates the transform
	void computeRGBD_ICP(const rgbdslam::featureMatch& msg);

	bool addEdgeToG2O(const LoadedEdge3D& edge, bool largeEdge, bool set_estimate);

	void optimizeGraph();


private:

	std::vector<graphnode> graphNodes;
	g2o::SparseOptimizer* optimizer_;


};


Eigen::Matrix4f EigenfromTf(tf::Transform trans);

g2o::SE3Quat eigen2G2O(const Eigen::Matrix4d eigen_mat);


#endif /* RGBD_ICP_H_ */
