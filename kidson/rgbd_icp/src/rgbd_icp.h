/*
 * rgbd_icp.h
 *
 *  Created on: Feb 13, 2012
 *      Author: ross
 */

#ifndef RGBD_ICP_H_
#define RGBD_ICP_H_

#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "rgbdslam/featureMatch.h"
#include "rgbdslam/match.h"
#include "pcl_ros/transforms.h"
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include "graphnode.h"

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

	// Callback that processes data
	void processRGBD_ICP(const rgbdslam::featureMatch& msg);

private:

	std::vector<graphnode> graphNodes;
	g2o::SparseOptimizer* optimizer_;


};



#endif /* RGBD_ICP_H_ */
