/*
 * scan.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: vsu
 */


#include <training.h>


int main(int argc, char** argv)
{
  size_t min_points_in_segment = 300;
  int num_clusters = 5;
  std::string input_dir = "data/scans/";
  std::string output_file = "data/codebook.yaml";

  std::vector<featureType> features;
  std::vector<Eigen::Vector4f> centroids;
  std::vector<std::string> classes;

  std::vector<featureType> cluster_centers;
  std::vector<int> cluster_labels;

  std::vector<std::string> files_to_process;
  get_files_to_process(input_dir, files_to_process);

  for (size_t i = 0; i < files_to_process.size(); i++)
  {
    append_segments_from_file(files_to_process[i], features, centroids, classes, min_points_in_segment);
  }

  cluster_features(features, num_clusters, cluster_centers, cluster_labels);

  std::map<featureType, std::map<std::string, std::vector<Eigen::Vector4f> > > codebook;

  create_codebook(features, centroids, classes, cluster_centers, cluster_labels, codebook);

  save_codebook(output_file, codebook);

  return 0;
}
