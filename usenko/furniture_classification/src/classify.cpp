/*
 * classify.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: vsu
 */

#include <training.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>



template class pcl::KdTreeFLANN<featureType>;
template class pcl::search::KdTree<featureType>;

int main(int argc, char** argv)
{

  std::string database_file_name = "data/codebook.yaml";
  std::string scene_file_name =
      "data/test/Chairs/Aluminium_Group_EA_119_0000F152-centered/rotation30_distance4_tilt-15_shift0.pcd";
  int min_points_in_segment = 300;

  std::map<featureType, std::map<std::string, std::vector<Eigen::Vector4f> > > codebook;
  pcl::PointCloud<featureType> feature_cloud;
  load_codebook(database_file_name, codebook, feature_cloud);

  std::vector<featureType> features;
  std::vector<Eigen::Vector4f> centroids;
  std::vector<std::string> classes;

  append_segments_from_file(scene_file_name, features, centroids, classes, min_points_in_segment);

  classes.clear();

  std::cout << "Found segments " << features.size() << " Cluster size " << feature_cloud.points.size() << std::endl;

  pcl::search::KdTree<featureType> aa;

  //pcl::NNClassification<featureType> a;

  //a.setTrainingFeatures(feature_cloud.makeShared());

  //pcl::KdTree<featureType>::Ptr tree;
  //typename pcl::CustomPointRepresentation<featureType>::Ptr cpr (new pcl::CustomPointRepresentation<featureType> (INT_MAX, 0));
  //tree.reset(new pcl::KdTreeFLANN<featureType>);

  return 0;
}


