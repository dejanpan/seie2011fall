/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: transformation_estimation_lm.hpp 3041 2011-11-01 04:44:41Z rusu $
 *
 */
#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_FEATURE_MATCHES_HPP_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_FEATURE_MATCHES_HPP_

#include "pcl/registration/transformation_estimation_lm.h"
#include "pcl/registration/warp_point_rigid.h"
#include "pcl/registration/warp_point_rigid_6d.h"
#include "pcl/registration/distances.h"
#include <unsupported/Eigen/NonLinearOptimization>


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> int
pcl::registration::TransformationEstimationFeatureMatches<PointSource, PointTarget>::OptimizationFunctorWithIndices::operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
  const PointCloud<PointSource> & src_points = *estimator_->tmp_src_;
  const PointCloud<PointTarget> & tgt_points = *estimator_->tmp_tgt_;
  const std::vector<int> & src_indices = *estimator_->tmp_idx_src_;
  const std::vector<int> & tgt_indices = *estimator_->tmp_idx_tgt_;

  // Initialize the warp function with the given parameters
  Eigen::VectorXf params = x.cast<float> ();
  estimator_->warp_point_->setParam (params);

  // Transform each source point and compute its distance to the corresponding target point
  for (int i = 0; i < m_values; ++i)
  {
    const PointSource & p_src = src_points.points[src_indices[i]];
    const PointTarget & p_tgt = tgt_points.points[tgt_indices[i]];

    // Transform the source point based on the current warp parameters
    PointSource p_src_warped;
    estimator_->warp_point_->warpPoint (p_src, p_src_warped);
    
    if (i < (src_indices.size() - estimator_->mPointsFeatures))
    	fvec[i] = sqrt((1 - featureErrorWeight) * (1/estimator_->mPointsDense) ) * estimator_->computeDistancePointToPlane (p_src_warped, p_tgt);
    else
    	fvec[i] = sqrt(featureErrorWeight * (1/estimator_->mPointsFeature) ) * estimator_->computeDistance (p_src_warped, p_tgt);
  }
  return (0);
}


//#define PCL_INSTANTIATE_TransformationEstimationLM(T,U) template class PCL_EXPORTS pcl::registration::TransformationEstimationLM<T,U>;

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_FEATURE_MATCHES_HPP_ */
