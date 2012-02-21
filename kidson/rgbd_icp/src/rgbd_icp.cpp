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
	graphNodes.resize(0);
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

/** @brief Helper function to convert Eigen transformation to tf */
Eigen::Matrix4f EigenfromTf(tf::Transform trans)
{
	Eigen::Matrix4f eignMat;
	eignMat(0,3) = trans.getOrigin().getX();
	eignMat(1,3) = trans.getOrigin().getY();
	eignMat(2,3) = trans.getOrigin().getZ();
	eignMat(0,0) = trans.getBasis().getRow(0).getX();
	eignMat(0,1) = trans.getBasis().getRow(0).getY();
	eignMat(0,2) = trans.getBasis().getRow(0).getZ();
	eignMat(1,0) = trans.getBasis().getRow(1).getX();
	eignMat(1,1) = trans.getBasis().getRow(1).getY();
	eignMat(1,2) = trans.getBasis().getRow(1).getZ();
	eignMat(2,0) = trans.getBasis().getRow(2).getX();
	eignMat(2,1) = trans.getBasis().getRow(2).getY();
	eignMat(2,2) = trans.getBasis().getRow(2).getZ();
	eignMat(3,3) = 1;
	//ROS_INFO("trans: %f, %f, %f %f | %f, %f, %f %f | %f, %f, %f %f", eignMat(0,0), eignMat(0,1), eignMat(0,2), eignMat(0,3), eignMat(1,0), eignMat(1,1), eignMat(1,2), eignMat(1,3), eignMat(2,0), eignMat(2,1), eignMat(2,2), eignMat(2,3));
    return eignMat;
}

g2o::SE3Quat eigen2G2O(const Eigen::Matrix4d eigen_mat) {
  Eigen::Affine3d eigen_transform(eigen_mat);
  Eigen::Quaterniond eigen_quat(eigen_transform.rotation());
  Eigen::Vector3d translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
  g2o::SE3Quat result(eigen_quat, translation);

  return result;
}


void rgbd_icp::processRGBD_ICP(const rgbdslam::featureMatch& msg)
{
	std::vector<cv::DMatch> inliers;
	//ROS_INFO("translation %f %f %f", msg.featureTransform.translation.x, msg.featureTransform.translation.y, msg.featureTransform.translation.z);
	std::vector<rgbdslam::match> local_matches = msg.matches;
  	for(std::vector<rgbdslam::match>::iterator iterator_ = local_matches.begin(); iterator_ != local_matches.end(); ++iterator_)
  	{
  		cv::DMatch oneMatch;
  		oneMatch.queryIdx = iterator_->queryId;
  		oneMatch.trainIdx = iterator_->trainId;
  		oneMatch.imgIdx = iterator_->imgId;
  		oneMatch.distance = iterator_->distance;
  		inliers.push_back(oneMatch);
  		//ROS_INFO("qidx: %d tidx: %d iidx: %d dist: %f", iterator_->queryId, iterator_->trainId, iterator_->imgId, iterator_->distance);
  	}

  	// First add the point cloud to the graph
  	graphnode newNode;
  	if(graphNodes.size() == 0)		// first node
  	{
  		//init graph node structure
  		newNode.id = 0;
  		newNode.pointCloud = msg.sourcePointcloud;
  		graphNodes.push_back(newNode);

  		//init optimizer
  		g2o::VertexSE3* reference_pose = new g2o::VertexSE3;
  		reference_pose->setId(0);
  		reference_pose->setEstimate(g2o::SE3Quat());
  		reference_pose->setFixed(true);//fix at origin
  		optimizer_->addVertex(reference_pose);
  	}
  	newNode.id = graphNodes.size();
	newNode.pointCloud = msg.targetPointcloud;
	graphNodes.push_back(newNode);

	//Create g2o 3d edge
	LoadedEdge3D edge;
    edge.id1 = newNode.id-1;
    edge.id2 = newNode.id;
    tf::Transform trans;
    tf::transformMsgToTF(msg.featureTransform,trans);
    Eigen::Matrix4f tempTrafo = EigenfromTf(trans);
    edge.mean = eigen2G2O(tempTrafo.cast<double>());//we insert an edge between the frames

    double w = (double)inliers.size();
    edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity()*(w*w);

    addEdgeToG2O(edge, true, true);
    optimizeGraph();
}


bool rgbd_icp::addEdgeToG2O(const LoadedEdge3D& edge, bool largeEdge, bool set_estimate) {

    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(edge.id1));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(edge.id2));

    //std::stringstream buffer;
    //std::streambuf * old = std::cout.rdbuf(buffer.rdbuf());
    ROS_INFO_STREAM("edge mean:" << edge.mean);
    //std::string text = buffer.str(); // text will now contain "Bla\n"

    //ROS_INFO("edge mean: %s", edge.mean.to_homogenious_matrix().Matrix(1,2,1));

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
    g2o_edge->setInformation(edge.informationMatrix);
    optimizer_->addEdge(g2o_edge);
    return true;
}

void rgbd_icp::optimizeGraph(){
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
