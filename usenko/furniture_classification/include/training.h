/*
 * training.h
 *
 *  Created on: Apr 25, 2012
 *      Author: vsu
 */

#ifndef TRAINING_H_
#define TRAINING_H_

#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/sgf6.h>
#include <pcl/features/sgf4.h>
#include <opencv2/core/core.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

namespace pcl
{
template<int N>
  bool operator <(const Histogram<N> & f, const Histogram<N> & s)
  {
    std::vector<float> v_f(f.histogram, f.histogram + N), v_s(s.histogram, s.histogram + N);
    return v_f < v_s;
  }
}

const int featureLength = pcl::SGF4_SIZE;
typedef pcl::Histogram<featureLength> featureType;
typedef pcl::SGF4Estimation<pcl::PointXYZ, featureType> featureEstimation;

void filter_segments(const std::vector<std::vector<int> > & segment_indices,
                     std::vector<std::vector<int> > & new_segment_indices, size_t min_points_in_segment);

cv::Mat transform_to_mat(const std::vector<featureType> & features);
void transform_to_features(const cv::Mat & mat, std::vector<featureType> & features);

void append_segments_from_file(const std::string & filename, std::vector<featureType> & features, std::vector<
    Eigen::Vector4f> & centroids, std::vector<std::string> & classes, size_t min_points_in_segment);

void get_files_to_process(const std::string & input_dir, std::vector<std::string> & files_to_process);

void cluster_features(const std::vector<featureType> & features, int num_clusters,
                      std::vector<featureType> & cluster_centers, std::vector<int> & cluster_labels);

void create_codebook(const std::vector<featureType> & features, const std::vector<Eigen::Vector4f> & centroids,
                     const std::vector<std::string> & classes, const std::vector<featureType> & cluster_centers,
                     const std::vector<int> & cluster_labels, std::map<featureType, std::map<std::string, std::vector<
                         Eigen::Vector4f> > > & codebook);

void save_codebook(const std::string & filename, const std::map<featureType, std::map<std::string, std::vector<
    Eigen::Vector4f> > > & codebook);

#endif /* TRAINING_H_ */
