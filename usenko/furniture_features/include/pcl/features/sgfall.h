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
 */

#ifndef PCL_FEATURES_SGFALL_H_
#define PCL_FEATURES_SGFALL_H_

#include <pcl/features/feature.h>
#include <pcl/features/sgf1.h>
#include <pcl/features/sgf2.h>
#include <pcl/features/sgf3.h>
#include <pcl/features/sgf4.h>
#include <pcl/features/sgf5.h>
#include <pcl/features/sgf6.h>
#include <pcl/features/sgf7.h>
#include <pcl/features/sgf8.h>
#include <pcl/features/sgf9.h>

namespace pcl
{
const int SGFALL_SIZE = 25;

template<typename PointInT, typename PointOutT>
  class SGFALLEstimation : public Feature<PointInT, PointOutT>
  {

  public:

    using Feature<PointInT, PointOutT>::feature_name_;
    using Feature<PointInT, PointOutT>::input_;
    using Feature<PointInT, PointOutT>::surface_;
    using Feature<PointInT, PointOutT>::indices_;
    using Feature<PointInT, PointOutT>::search_parameter_;
    using Feature<PointInT, PointOutT>::tree_;
    using Feature<PointInT, PointOutT>::k_;

    typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
    typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;

    /** \brief Empty constructor. */
    SGFALLEstimation()
    {
      feature_name_ = "SGFALLEstimation";
      k_ = 1;
    }
    ;

    /////////////////////////////////////////////////////////////////////////////
    void computeFeature(PointCloudOut & sgfs)
    {
      size_t feature_counter = 0;

      sgfs.width = 1;
      sgfs.height = 1;
      sgfs.points.resize(1);

      /////////////////////////////////////////////////////////////////////////////////////
      // Feature 1

      pcl::PointCloud<pcl::Histogram<pcl::SGF1_SIZE> >::Ptr
                                                            sgf1s(
                                                                  new pcl::PointCloud<pcl::Histogram<pcl::SGF1_SIZE> >());
      pcl::SGF1Estimation<PointInT, pcl::Histogram<pcl::SGF1_SIZE> > sgf1;
      sgf1.setInputCloud(input_);
      sgf1.setIndices(indices_);
      sgf1.setSearchMethod(tree_);
      sgf1.setKSearch(k_);
      sgf1.compute(*sgf1s);

      for (int n = 0; n < pcl::SGF1_SIZE; ++n)
      {
        sgfs.points[0].histogram[feature_counter + n] = sgf1s->points[0].histogram[n];
      }
      feature_counter += pcl::SGF1_SIZE;

      /////////////////////////////////////////////////////////////////////////////////////
      // Feature 2

      pcl::PointCloud<pcl::Histogram<pcl::SGF2_SIZE> >::Ptr
                                                            sgf2s(
                                                                  new pcl::PointCloud<pcl::Histogram<pcl::SGF2_SIZE> >());
      pcl::SGF2Estimation<PointInT, pcl::Histogram<pcl::SGF2_SIZE> > sgf2;
      sgf2.setInputCloud(input_);
      sgf2.setIndices(indices_);
      sgf2.setKSearch(k_);
      sgf2.compute(*sgf2s);

      for (int n = 0; n < pcl::SGF2_SIZE; ++n)
      {
        sgfs.points[0].histogram[feature_counter + n] = sgf2s->points[0].histogram[n];
      }
      feature_counter += pcl::SGF2_SIZE;

      /////////////////////////////////////////////////////////////////////////////////////
      // Feature 3

      pcl::PointCloud<pcl::Histogram<pcl::SGF3_SIZE> >::Ptr
                                                            sgf3s(
                                                                  new pcl::PointCloud<pcl::Histogram<pcl::SGF3_SIZE> >());
      pcl::SGF3Estimation<PointInT, pcl::Histogram<pcl::SGF3_SIZE> > sgf3;
      sgf3.setInputCloud(input_);
      sgf3.setIndices(indices_);
      sgf3.compute(*sgf3s);

      for (int n = 0; n < pcl::SGF3_SIZE; ++n)
      {
        sgfs.points[0].histogram[feature_counter + n] = sgf3s->points[0].histogram[n];
      }
      feature_counter += pcl::SGF3_SIZE;

      /////////////////////////////////////////////////////////////////////////////////////
      // Feature 4

      pcl::PointCloud<pcl::Histogram<pcl::SGF4_SIZE> >::Ptr
                                                            sgf4s(
                                                                  new pcl::PointCloud<pcl::Histogram<pcl::SGF4_SIZE> >());
      pcl::SGF4Estimation<PointInT, pcl::Histogram<pcl::SGF4_SIZE> > sgf4;
      sgf4.setInputCloud(input_);
      sgf4.setIndices(indices_);
      sgf4.compute(*sgf4s);

      for (int n = 0; n < pcl::SGF4_SIZE; ++n)
      {
        sgfs.points[0].histogram[feature_counter + n] = sgf4s->points[0].histogram[n];
      }
      feature_counter += pcl::SGF4_SIZE;

      /////////////////////////////////////////////////////////////////////////////////////
      // Feature 5

      pcl::PointCloud<pcl::Histogram<pcl::SGF5_SIZE> >::Ptr
                                                            sgf5s(
                                                                  new pcl::PointCloud<pcl::Histogram<pcl::SGF5_SIZE> >());
      pcl::SGF5Estimation<PointInT, pcl::Histogram<pcl::SGF5_SIZE> > sgf5;
      sgf5.setInputCloud(input_);
      sgf5.setIndices(indices_);
      sgf5.compute(*sgf5s);

      for (int n = 0; n < pcl::SGF5_SIZE; ++n)
      {
        sgfs.points[0].histogram[feature_counter + n] = sgf5s->points[0].histogram[n];
      }
      feature_counter += pcl::SGF5_SIZE;

      /////////////////////////////////////////////////////////////////////////////////////
      // Feature 6

      pcl::PointCloud<pcl::Histogram<pcl::SGF6_SIZE> >::Ptr
                                                            sgf6s(
                                                                  new pcl::PointCloud<pcl::Histogram<pcl::SGF6_SIZE> >());
      pcl::SGF6Estimation<PointInT, pcl::Histogram<pcl::SGF6_SIZE> > sgf6;
      sgf6.setInputCloud(input_);
      sgf6.setIndices(indices_);
      sgf6.compute(*sgf6s);

      for (int n = 0; n < pcl::SGF6_SIZE; ++n)
      {
        sgfs.points[0].histogram[feature_counter + n] = sgf6s->points[0].histogram[n];
      }
      feature_counter += pcl::SGF6_SIZE;

      /////////////////////////////////////////////////////////////////////////////////////
      // Feature 7

      pcl::PointCloud<pcl::Histogram<pcl::SGF7_SIZE> >::Ptr
                                                            sgf7s(
                                                                  new pcl::PointCloud<pcl::Histogram<pcl::SGF7_SIZE> >());
      pcl::SGF7Estimation<PointInT, pcl::Histogram<pcl::SGF7_SIZE> > sgf7;
      sgf7.setInputCloud(input_);
      sgf7.setIndices(indices_);
      sgf7.compute(*sgf7s);

      for (int n = 0; n < pcl::SGF7_SIZE; ++n)
      {
        sgfs.points[0].histogram[feature_counter + n] = sgf7s->points[0].histogram[n];
      }
      feature_counter += pcl::SGF7_SIZE;

      /////////////////////////////////////////////////////////////////////////////////////
      // Feature 8

      pcl::PointCloud<pcl::Histogram<pcl::SGF8_SIZE> >::Ptr
                                                            sgf8s(
                                                                  new pcl::PointCloud<pcl::Histogram<pcl::SGF8_SIZE> >());
      pcl::SGF8Estimation<PointInT, pcl::Histogram<pcl::SGF8_SIZE> > sgf8;
      sgf8.setInputCloud(input_);
      sgf8.setIndices(indices_);
      sgf8.compute(*sgf8s);

      for (int n = 0; n < pcl::SGF8_SIZE; ++n)
      {
        sgfs.points[0].histogram[feature_counter + n] = sgf8s->points[0].histogram[n];
      }
      feature_counter += pcl::SGF8_SIZE;

      /////////////////////////////////////////////////////////////////////////////////////
      // Feature 9

      pcl::PointCloud<pcl::Histogram<pcl::SGF9_SIZE> >::Ptr
                                                            sgf9s(
                                                                  new pcl::PointCloud<pcl::Histogram<pcl::SGF9_SIZE> >());
      pcl::SGF9Estimation<PointInT, pcl::Histogram<pcl::SGF9_SIZE> > sgf9;
      sgf9.setInputCloud(input_);
      sgf9.setIndices(indices_);
      sgf9.compute(*sgf9s);

      for (int n = 0; n < pcl::SGF9_SIZE; ++n)
      {
        sgfs.points[0].histogram[feature_counter + n] = sgf9s->points[0].histogram[n];
      }
      feature_counter += pcl::SGF9_SIZE;

    }
    /////////////////////////////////////////////////////////////////////////////


  private:

    /** \brief Make the computeFeature (&Eigen::MatrixXf); inaccessible from outside the class
     * \param[out] output the output point cloud
     */
    void computeFeatureEigen(pcl::PointCloud<Eigen::MatrixXf> &)
    {
    }
  };
}

#endif  //#ifndef PCL_FEATURES_SGF9_H_
