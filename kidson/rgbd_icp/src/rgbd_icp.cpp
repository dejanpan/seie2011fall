/*
 * rgbd.cpp
 *
 *  Created on: Feb 13, 2012
 *      Author: ross
 */

#include "rgbd_icp.h"

//typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*>     VertexIDMap;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;

rgbd_icp::rgbd_icp()
{
	// allocating the optimizer
	optimizer_ = new g2o::SparseOptimizer();
	optimizer_->setVerbose(true);
	//SlamLinearSolver* linearSolver = new SlamLinearSolver();
	SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
	//SlamLinearPCGSolver* linearSolver = new SlamLinearPCGSolver();
	linearSolver->setBlockOrdering(false);
	SlamBlockSolver* solver = new SlamBlockSolver(optimizer_, linearSolver);
	optimizer_->setSolver(solver);
}


void rgbd_icp::processRGBD_ICP(const rgbdslam::featureMatch& msg)
{
	//ROS_INFO("translation %f %f %f", msg.featureTransform.translation.x, msg.featureTransform.translation.y, msg.featureTransform.translation.z);
	std::vector<rgbdslam::match> local_matches = msg.matches;
  	for(std::vector<rgbdslam::match>::iterator iterator_ = local_matches.begin(); iterator_ != local_matches.end(); ++iterator_) {
  		//ROS_INFO("qidx: %d tidx: %d iidx: %d dist: %f", iterator_->queryId, iterator_->trainId, iterator_->imgId, iterator_->distance);
  	}

  	// First add the point cloud to the graph
  	graphnode newNode;
  	if(graphNodes.size() == 0)		// first node
  	{
  		newNode.id = 0;
  		newNode.pointCloud = msg.sourcePointcloud;
  		graphNodes.push_back(newNode);
  	}
  	newNode.id = graphNodes.size();
	newNode.pointCloud = msg.targetPointcloud;
	graphNodes.push_back(newNode);


}

