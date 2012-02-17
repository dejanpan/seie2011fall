/*
 * rgbd.cpp
 *
 *  Created on: Feb 13, 2012
 *      Author: Ross
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

	LoadedEdge3D edge;
    edge.id1 = newNode.id-1;//and we have a valid transformation
    edge.id2 = newNode.id; //since there are enough matching features,
    //edge.mean = eigen2G2O(   (msg.featureTransform) This is geometry msg.  convert to eigenmatrix   .cast<double>());//we insert an edge between the frames

    addEdgeToG2O(edge, true, true);
}


bool rgbd_icp::addEdgeToG2O(const LoadedEdge3D& edge, bool largeEdge, bool set_estimate) {

    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(edge.id1));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(edge.id2));

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
        //latest_transform_ = g2o2QMatrix(v1->estimate());
    }
    else if (!v2 && v1) {
        v2 = new g2o::VertexSE3;
        assert(v2);
        v2->setId(edge.id2);
        v2->setEstimate(v1->estimate() * edge.mean);
        optimizer_->addVertex(v2);
        //latest_transform_ = g2o2QMatrix(v2->estimate());
    }
    else if(set_estimate){
        v2->setEstimate(v1->estimate() * edge.mean);
    }
    g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3;
    g2o_edge->vertices()[0] = v1;
    g2o_edge->vertices()[1] = v2;
    g2o_edge->setMeasurement(edge.mean);
    g2o_edge->setInverseMeasurement(edge.mean.inverse());
    //g2o_edge->setInformation(edge.informationMatrix);
    optimizer_->addEdge(g2o_edge);
    return true;
}

void rgbd_icp::optimizeGraph(int iter){
    int iterations = 2;

    ROS_WARN("Starting Optimization");
    std::string bagfile_name = "optimizer_output";
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


    //g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(optimizer_->vertices().size()-1));

    //computed_motion_ =  g2o2TF(v->estimate());
    //latest_transform_ = g2o2QMatrix(v->estimate());
}
