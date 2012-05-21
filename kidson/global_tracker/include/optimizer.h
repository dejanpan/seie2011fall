/*
 * optimizer.h
 *
 *  Created on: Jul 21, 2011
 *      Author: Nikolas Engelhard
 */

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include "ros/ros.h"
#include "rosbag/bag.h"

#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <frame_common/stereo.h>
#include <frame_common/camparams.h>
//#include <g2o/core/graph_optimizer_sparse.h>
//#include <g2o/solvers/csparse/linear_solver_csparse.h>
//#include <g2o/solvers/pcg/linear_solver_pcg.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/linear_solver.h>
//
//#include <g2o/types/slam3d/vertex_depthcam.h> // definition of camera (pose & intrinsic)
//#include <g2o/types/slam3d/vertex_trackxyz.h> // definition of landmark
//
//#include <g2o/types/slam3d/edge_project_disparity.h> // oberservation of landmarl (cam <-> lm)
//#include <g2o/types/slam3d/edge_dc2dc.h>  // relative between cams (e.g. odo:  cam <-> cam)

#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>


#include "cluster_manager.h"
#include "node_definitions.h"


namespace cluster_optimization {

void optimize_cluster(Cluster_manager& cm, cluster* cl);
void optimize_with_landmarks(Cluster_manager& cm);


void updatePoses(Cluster_manager& cm, g2o::SparseOptimizer* optimizer, set<uint>& optimized_nodes);

void setPoseWithConstantvelocity(Cluster_manager& cm, uint node_id);


void populateOptimizer(Cluster_manager& cm, cluster* cl,g2o::SparseOptimizer* optimizer,
                       uint vertexCounter, set<uint>& optimized_nodes);

void initOptimizer(g2o::SparseOptimizer* optimizer, frame_common::CamParams cam);

}


#endif /* OPTIMIZER_H_ */
