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

#include "pcl/registration/transformation_estimation_feature_matches.h"
#include "pcl/registration/warp_point_rigid.h"
#include "pcl/registration/warp_point_rigid_6d.h"
#include "pcl/registration/distances.h"
#include <unsupported/Eigen/NonLinearOptimization>

template <typename PointSource, typename PointTarget> inline void
pcl::registration::TransformationEstimationFeatureMatches<PointSource, PointTarget>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const std::vector<int> &indices_tgt,
    Eigen::Matrix4f &transformation_matrix)
{
	std::cerr << "pcl::registration::TransformationEstimationFeatureMatches<PointSource, PointTarget>::estimateRigidTransformation \n";
  if (indices_src.size () != indices_tgt.size ())
  {
    PCL_ERROR ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)indices_src.size (), (unsigned long)indices_tgt.size ());
    return;
  }

  if (indices_src.size () < 4)     // need at least 4 samples
  {
    PCL_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] ");
    PCL_ERROR ("Need at least 4 points to estimate a transform! Source and target have %lu points!",
               (unsigned long)indices_src.size ());
    return;
  }

  // If no warp function has been set, use the default (WarpPointRigid6D)
  if (!warp_point_)
    warp_point_.reset (new WarpPointRigid6D<PointSource, PointTarget>);

  int n_unknowns = warp_point_->getDimension ();  // get dimension of unknown space
  int m = indices_src.size ();
  Eigen::VectorXd x(n_unknowns);
  x.setConstant (n_unknowns, 0);

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;
  tmp_idx_src_ = &indices_src;
  tmp_idx_tgt_ = &indices_tgt;

  OptimizationFunctorWithIndices functor (n_unknowns, m, this);
  Eigen::NumericalDiff<OptimizationFunctorWithIndices> num_diff (functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctorWithIndices> > lm (num_diff);
  int info = lm.minimize (x);

  // Compute the norm of the residuals
  PCL_DEBUG ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation]");
  PCL_DEBUG ("LM solver finished with exit code %i, having a residual norm of %g. \n", info, lm.fvec.norm ());
  PCL_DEBUG ("Final solution: [%f", x[0]);
  for (int i = 1; i < n_unknowns; ++i)
    PCL_DEBUG (" %f", x[i]);
  PCL_DEBUG ("]\n");

  // Return the correct transformation
  Eigen::VectorXf params = x.cast<float> ();
  warp_point_->setParam (params);
  transformation_matrix = warp_point_->getTransform ();

  tmp_src_ = NULL;
  tmp_tgt_ = NULL;
  tmp_idx_src_ = tmp_idx_tgt_ = NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> int
pcl::registration::TransformationEstimationFeatureMatches<PointSource, PointTarget>::OptimizationFunctorWithIndices::operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
  const PointCloud<PointSource> & src_points = *estimator_->tmp_src_;
  const PointCloud<PointTarget> & tgt_points = *estimator_->tmp_tgt_;
  const std::vector<int> & src_indices = *estimator_->tmp_idx_src_;
  const std::vector<int> & tgt_indices = *estimator_->tmp_idx_tgt_;

  int mPointsDense = m_values - estimator_->mPointsFeatures;
  // std::cerr << "mvals: " << m_values << " mFeatures: " << estimator_->mPointsFeatures << "\n"
   //<< "src indicies size: " << src_indices.size() << " target indices size: " << tgt_indices.size() << "\n"
   //<< "source points:" << src_points.size() << " target points size: " << tgt_points.size() << "\n";

  // Initialize the warp function with the given parameters
  Eigen::VectorXf params = x.cast<float> ();
  estimator_->warp_point_->setParam (params);

  //std::cerr << " ###################################BEGIN distances#####################################################" << "\n";
  //std::cerr << "correspondences dense:" << mPointsDense << "\n";

  // Transform each source point and compute its distance to the corresponding target point
  for (int i = 0; i < m_values; ++i)
  {
    const PointSource & p_src = src_points.points[src_indices[i]];
    const PointTarget & p_tgt = tgt_points.points[tgt_indices[i]];

    // Transform the source point based on the current warp parameters
    PointSource p_src_warped;
    estimator_->warp_point_->warpPoint (p_src, p_src_warped);
    
    if (i < mPointsDense)
    {
    	fvec[i] = sqrt((1 - estimator_->featureErrorWeight) * (1.0/(double)mPointsDense) ) * estimator_->computeDistancePointToPlane (p_src_warped, p_tgt);
    	//if ( i == 100)	// just take one point
    	//{
    	//	std::cerr << "i : " << i << "sqrt((1 - estimator_->featureErrorWeight) * (1.0/(double)mPointsDense)) : "
    	//			<< sqrt((1 - estimator_->featureErrorWeight) * (1.0/(double)mPointsDense) )
    	//			<< " estimator_->computeDistancePointToPlane (p_src_warped, p_tgt) : "
    	//			<<  estimator_->computeDistancePointToPlane (p_src_warped, p_tgt) << "\n";
    	//}
    }
    else
    {
    	//std::cerr << " Source indice (" << i << "): " << src_indices[i] << "\n";
    	//std::cerr << " Target indice (" << i << "): " << tgt_indices[i] << "\n";
    	fvec[i] = sqrt(estimator_->featureErrorWeight * (1.0/(double)estimator_->mPointsFeatures) ) * estimator_->computeDistance (p_src_warped, p_tgt);
    	//std::cerr << "source point ind:" << i << " point:" << src_indices[i] << " : " << src_points.points[src_indices[i]].x << ", " << src_points.points[src_indices[i]].y << ", " << src_points.points[src_indices[i]].z << "\n";
    	//std::cerr << "target point ind:" << i << " point:" << tgt_indices[i] << " : " << tgt_points.points[tgt_indices[i]].x << ", " << tgt_points.points[tgt_indices[i]].y << ", " << tgt_points.points[tgt_indices[i]].z << "\n";
    	//std::cerr << " alpha : " << estimator_->featureErrorWeight << "\n ";
    	//std::cerr << " 1/estimator_->mPointsFeatures : " << (1.0/(double)estimator_->mPointsFeatures) << "\n ";
    	//std::cerr << " estimator_->computeDistance (p_src_warped, p_tgt) : " << estimator_->computeDistance (p_src_warped, p_tgt) << "\n ";
    	//std::cerr << " i : " << i << " fvec[i]: " << fvec[i] << " \n ";
    	//std::cerr << " p_src : " << p_src << " p_tgt" << p_tgt << "\n";
    }
  }
  std::cerr << " Error(distance): " << fvec.sum() << "\n";
  //std::cerr << "fvec: " << fvec.sum() << "\n";
  if ((fvec.sum() > 1e+5) || (fvec.sum() < -1e+5))
		  std::cerr << "##### FATAL ICP ERROR  ######    fvec.sum():" << fvec.sum() << "\n";
  return (0);
}



//#define PCL_INSTANTIATE_TransformationEstimationLM(T,U) template class PCL_EXPORTS pcl::registration::TransformationEstimationLM<T,U>;

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_FEATURE_MATCHES_HPP_ */
