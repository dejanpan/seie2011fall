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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/sgfall.h>
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

const int featureLength = pcl::SGFALL_SIZE;
typedef pcl::Histogram<featureLength> featureType;
typedef pcl::SGFALLEstimation<pcl::PointXYZ, featureType> featureEstimation;
typedef std::map<featureType, std::map<std::string, pcl::PointCloud<pcl::PointXYZ> > > databaseType;

void filter_segments(const std::vector<std::vector<int> > & segment_indices,
                     std::vector<std::vector<int> > & new_segment_indices, size_t min_points_in_segment);

cv::Mat transform_to_mat(const std::vector<featureType> & features);
void transform_to_features(const cv::Mat & mat, std::vector<featureType> & features);

void append_segments_from_file(const std::string & filename, std::vector<featureType> & features, pcl::PointCloud<
    pcl::PointXYZ> & centroids, std::vector<std::string> & classes, size_t min_points_in_segment);

void get_files_to_process(const std::string & input_dir, std::vector<std::string> & files_to_process);

void cluster_features(const std::vector<featureType> & features, int num_clusters,
                      std::vector<featureType> & cluster_centers, std::vector<int> & cluster_labels);

void create_codebook(const std::vector<featureType> & features, const pcl::PointCloud<pcl::PointXYZ> & centroids,
                     const std::vector<std::string> & classes, const std::vector<featureType> & cluster_centers,
                     const std::vector<int> & cluster_labels, databaseType & codebook);

void save_codebook(const std::string & filename, const databaseType & database);

void load_codebook(const std::string & filename, databaseType & codebook, pcl::PointCloud<featureType> & feature_cloud);

template<int N>
  YAML::Emitter& operator <<(YAML::Emitter& out, const pcl::Histogram<N> & h)
  {
    out << YAML::Flow << YAML::BeginSeq;
    for (int j = 0; j < N; j++)
    {
      out << h.histogram[j];
    }
    out << YAML::EndSeq << YAML::Block;
    return out;
  }

template<int N>
  void operator >>(const YAML::Node& node, pcl::Histogram<N> & h)
  {
    for (int j = 0; j < N; j++)
    {
      node[j] >> h.histogram[j];
    }
  }

template<typename PointT>
  YAML::Emitter& operator <<(YAML::Emitter& out, const pcl::PointCloud<PointT> & cloud)
  {
    out << YAML::BeginSeq;
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
      out << YAML::Flow << YAML::BeginSeq << cloud.points[i].x << cloud.points[i].y << cloud.points[i].z
          << YAML::EndSeq << YAML::Block;
    }
    out << YAML::EndSeq;

    return out;
  }

template<typename PointT>
  void operator >>(const YAML::Node& node, pcl::PointCloud<PointT> & cloud)
  {
    cloud.clear();

    for (size_t i = 0; i < node.size(); i++)
    {
      PointT point;
      node[i][0] >> point.x;
      node[i][1] >> point.y;
      node[i][2] >> point.z;
      cloud.points.push_back(point);

    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
  }

template<int N>
  void normalizeFeatures(std::vector<pcl::Histogram<N> > & features)
  {
    pcl::Histogram<N> min, max;

    // Init max and min vales with first feature
    for (int j = 0; j < N; j++)
    {
      min.histogram[j] = features[0].histogram[j];
      max.histogram[j] = features[0].histogram[j];
    }

    // Find max and min values
    for (size_t i = 0; i < features.size(); i++)
    {
      for (int j = 0; j < N; j++)
      {
        if (features[i].histogram[j] < min.histogram[j])
          min.histogram[j] = features[i].histogram[j];
        if (features[i].histogram[j] > max.histogram[j])
          max.histogram[j] = features[i].histogram[j];
      }
    }

    // Normalize
    for (size_t i = 0; i < features.size(); i++)
    {
      for (int j = 0; j < N; j++)
      {
        features[i].histogram[j] = (features[i].histogram[j] - min.histogram[j])
            / (max.histogram[j] - min.histogram[j]);
      }
    }

  }

#endif /* TRAINING_H_ */
