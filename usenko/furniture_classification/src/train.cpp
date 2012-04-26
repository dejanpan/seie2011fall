/*
 * scan.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: vsu
 */

#include <pcl/console/parse.h>
#include <training.h>

int main(int argc, char** argv)
{

  if (argc < 5)
  {
    PCL_INFO ("Usage %s -input_dir /dir/with/pointclouds -output_file /where/to/put/database [options]\n", argv[0]);
    PCL_INFO (" * where options are:\n"
        "         -min_points_in_segment <X>  : set minimal number of points in segment to X. Default : 300\n"
        "         -num_clusters <X>           : set Number of clusters. Default : 5\n"
        "");
    return -1;
  }

  int min_points_in_segment = 300;
  int num_clusters = 5;
  std::string input_dir;
  std::string output_file;

  pcl::console::parse_argument(argc, argv, "-input_dir", input_dir);
  pcl::console::parse_argument(argc, argv, "-output_file", output_file);
  pcl::console::parse_argument(argc, argv, "-num_clusters", num_clusters);
  pcl::console::parse_argument(argc, argv, "-min_points_in_segment", min_points_in_segment);

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

  normalizeFeatures(features);

  cluster_features(features, num_clusters, cluster_centers, cluster_labels);

  std::map<featureType, std::map<std::string, std::vector<Eigen::Vector4f> > > codebook;

  create_codebook(features, centroids, classes, cluster_centers, cluster_labels, codebook);

  save_codebook(output_file, codebook);

  return 0;
}

