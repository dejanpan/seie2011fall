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
 * $Id: transformation_estimation_point_to_plane.h 3217 2011-11-21 10:42:11Z mdixon $
 *
 */
#ifndef PCL_REGISTRATION_TRANSFORMATION_FEATURE_MATCHES_H_
#define PCL_REGISTRATION_TRANSFORMATION_FEATURE_MATCHES_H_

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid.h>

namespace pcl
{
  namespace registration
  {
    /** @b TransformationEstimationPointToPlane uses Levenberg Marquardt optimization to find the
      * transformation that minimizes the point-to-plane distance between the given correspondences.
      *
      * \author Michael Dixon
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget>
    class TransformationEstimationFeatureMatches : public TransformationEstimationLM<PointSource, PointTarget>
    {
      public:
        typedef boost::shared_ptr<TransformationEstimationFeatureMatches<PointSource, PointTarget> > Ptr;
        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;
        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef PointIndices::Ptr PointIndicesPtr;
        typedef PointIndices::ConstPtr PointIndicesConstPtr;

        /*inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const std::vector<int> &indices_tgt,
            Eigen::Matrix4f &transformation_matrix);*/

        TransformationEstimationFeatureMatches () {};
        virtual ~TransformationEstimationFeatureMatches () {};

      protected:
        virtual double
        computeDistance (const PointSource &p_src, const PointTarget &p_tgt)
        { 
          // Compute the point-to-plane distance
          Vector4fMapConst s = p_src.getVector4fMap ();
          Vector4fMapConst t = p_tgt.getVector4fMap ();
          Vector4fMapConst n = p_tgt.getVector4fMap (); //p_tgt.getNormalVector4fMap ();
          return ((s - t).dot (n));
        }
        /** Generic functor for the optimization */
        template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
        struct Functor
        {
          typedef _Scalar Scalar;
          enum {
            InputsAtCompileTime = NX,
            ValuesAtCompileTime = NY
          };
          typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
          typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
          typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

          const int m_inputs, m_values;

          Functor () : m_inputs (InputsAtCompileTime), m_values (ValuesAtCompileTime) {}
          Functor (int inputs, int values) : m_inputs (inputs), m_values (values) {}

          int inputs () const { return m_inputs; }
          int values () const { return m_values; }
        };

        struct OptimizationFunctorWithIndices : Functor<double>
        {
          using Functor<double>::m_values;

          /** Functor constructor
            * \param n Number of unknowns to be solved
            * \param m Number of values
            * \param estimator pointer to the estimator object
            * \param distance distance computation function pointer
            */
          OptimizationFunctorWithIndices (int n, int m, TransformationEstimationFeatureMatches *estimator) :
            Functor<double> (n,m), estimator_(estimator) {}

          /** Fill fvec from x. For the current state vector x fill the f values
            * \param x state vector
            * \param fvec f values vector
            * \return 0
            */
          int operator () (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;

          TransformationEstimationFeatureMatches<PointSource, PointTarget> *estimator_;
        };
    };
  }
}

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_FEATURE_MATCHES_H_ */

