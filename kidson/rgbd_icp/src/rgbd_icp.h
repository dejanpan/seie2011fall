/*
 * rgbd_icp.h
 *
 *  Created on: Feb 13, 2012
 *      Author: ross
 */

#ifndef RGBD_ICP_H_
#define RGBD_ICP_H_

#include "edge.h"
#include "rgbdslam/featureMatch.h"
#include "graphnode.h"

#include "g2o/math_groups/se3quat.h"
#include "g2o/core/graph_optimizer_sparse.h"

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
