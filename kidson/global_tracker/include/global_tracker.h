/*
 * global_tracker.h
 *
 *  Created on: Sep 1, 2011
 *      Author: engelhar
 */

#ifndef GLOBAL_TRACKER_H_
#define GLOBAL_TRACKER_H_

#include "localposeestimator.h"
#include "featuretracker_datatypes.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/stereo_camera_model.h>

using namespace sensor_msgs;

class GlobalTracker : public LocalPoseEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  GlobalTracker(unsigned int priority, std::string *_sensorFrame, cv::Mat *image, cv::Mat *depthImage, image_geometry::PinholeCameraModel *cam);
  virtual ~GlobalTracker();

  // inherited from LocalPoseEstimator
  virtual void initialize();
  virtual bool processFrame(g2o::VertexSE3 *frameVertex);
  virtual bool insertEdges(g2o::OptimizableGraph *graph);
  virtual void visualize();
  virtual void makeKeyFrame(g2o::VertexSE3 *frameVertex);
private:

  cv::Mat *rgb_img;


};















#endif /* GLOBAL_TRACKER_H_ */
