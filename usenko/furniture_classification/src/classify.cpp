/*
 * classify.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: vsu
 */

#include <pcl/console/parse.h>
#include <training.h>



int main(int argc, char** argv)
{

  if (argc < 5)
  {
    PCL_INFO ("Usage %s -database_file_name /database.yaml -scene_file_name /scene.pcd [options]\n", argv[0]);
    PCL_INFO (" * where options are:\n"
        "         -num_neighbours <X>  : number of neighbours. Default : 1\n"
        "         -cell_size <X>  : cell size for grid. Default : 0.01\n"
        "         -window_size <X>  : window size for local maxima search. Default : 0.4\n"
        "         -local_maxima_threshold <X>  : threshold for local maxima search. Default : 0.4\n"
        "         -use_icp <X>  : use ICP refinement. Default : false\n"
        "         -icp_threshold <X>  : if ICP error < icp_threshold we assume model fits.  Default : 0.01\n"
        "         -num_rotations_icp <X>  : number of starting rotations for ICP. Default : 12\n"
        "");
    return -1;
  }

  std::string database_file_name = "database.yaml";
  std::string scene_file_name;
  int min_points_in_segment = 300;
  int num_neighbours = 1;
  float cell_size = 0.01;
  float window_size = 0.4;
  float local_maxima_threshold = 0.4;
  int num_rotations_icp = 12;
  bool use_icp = false;
  float icp_threshold = 0.01;

  pcl::console::parse_argument(argc, argv, "-database_file_name", database_file_name);
  pcl::console::parse_argument(argc, argv, "-scene_file_name", scene_file_name);
  pcl::console::parse_argument(argc, argv, "-num_neighbours", num_neighbours);
  pcl::console::parse_argument(argc, argv, "-cell_size", cell_size);
  pcl::console::parse_argument(argc, argv, "-window_size", window_size);
  pcl::console::parse_argument(argc, argv, "-local_maxima_threshold", local_maxima_threshold);
  pcl::console::parse_argument(argc, argv, "-use_icp", use_icp);
  pcl::console::parse_argument(argc, argv, "-icp_threshold", icp_threshold);
  pcl::console::parse_argument(argc, argv, "-num_rotations_icp", num_rotations_icp);


  // create directory structure
  std::vector<std::string> st;
  boost::split(st, scene_file_name, boost::is_any_of("/"), boost::token_compress_on);
  std::string scene_name = st.at(st.size() - 1);
  scene_name = scene_name.substr(0, scene_name.size() - 4);

  // Check if output directory exists
  boost::filesystem::path output_path(scene_name);
  if (!boost::filesystem::exists(output_path) || !boost::filesystem::is_directory(output_path))
  {
    if (!boost::filesystem::create_directories(output_path))
    {
      PCL_ERROR ("Error creating directory %s.\n", output_path.c_str ());
      return -1;
    }

    boost::filesystem::path debug_path(scene_name + "/debug");
    boost::filesystem::path result_path(scene_name + "/result");

    if (!boost::filesystem::create_directories(debug_path))
    {
      PCL_ERROR ("Error creating directory %s.\n", output_path.c_str ());
      return -1;
    }
    if (!boost::filesystem::create_directories(result_path))
    {
      PCL_ERROR ("Error creating directory %s.\n", output_path.c_str ());
      return -1;
    }

  }

  databaseType database;
  pcl::PointCloud<featureType>::Ptr feature_cloud(new pcl::PointCloud<featureType>());
  featureType min_train, max_train;
  std::map<std::string, std::vector<std::string> > class_to_full_pointcloud;
  load_codebook(database_file_name, database, *feature_cloud, min_train, max_train, class_to_full_pointcloud);

  std::vector<featureType> features;
  pcl::PointCloud<pcl::PointXYZ> centroids;
  std::vector<std::string> classes;
  pcl::PointXYZ min_bound, max_bound;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);

  append_segments_from_file(scene_file_name, features, centroids, classes, min_points_in_segment, *scene, &min_bound,
                            &max_bound);

  featureType min, max;
  normalizeFeatures(features, min, max, min_train, max_train);

  classes.clear();

  std::cout << "Found segments " << features.size() << " Cluster size " << feature_cloud->points.size() << std::endl;

  pcl::search::KdTree<featureType> feature_search;
  feature_search.setInputCloud(feature_cloud);

  std::map<std::string, pcl::PointCloud<pcl::PointXYZI> > votes;

  for (size_t i = 0; i < features.size(); i++)
  {
    std::vector<int> indices;
    std::vector<float> distances;
    feature_search.nearestKSearch(features[i], num_neighbours, indices, distances);

    for (size_t j = 0; j < indices.size(); j++)
    {
      for (std::map<std::string, pcl::PointCloud<pcl::PointXYZ> >::const_iterator it =
          database[feature_cloud->at(indices[j])].begin(); it != database[feature_cloud->at(indices[j])].end(); it++)
      {
        std::string class_name = it->first;
        pcl::PointCloud<pcl::PointXYZ> model_centers = it->second;
        pcl::PointCloud<pcl::PointXYZ> model_centers_transformed;
        pcl::PointCloud<pcl::PointXYZI> model_centers_transformed_weighted;

        Eigen::Affine3f transform;
        transform.setIdentity();
        transform.translate(centroids[i].getVector3fMap());
        pcl::transformPointCloud(model_centers, model_centers_transformed, transform);

        pcl::copyPointCloud(model_centers_transformed, model_centers_transformed_weighted);

        // TODO revise weighting function
        for (size_t i = 0; i < model_centers_transformed_weighted.size(); i++)
        {
          model_centers_transformed_weighted.points[i].intensity = (1.0 / distances[j]) * (1.0 / model_centers.size());
        }

        votes[class_name] += model_centers_transformed_weighted;
      }
    }

  }

  for (std::map<std::string, pcl::PointCloud<pcl::PointXYZI> >::const_iterator it = votes.begin(); it != votes.end(); it++)
  {
    std::string class_name = it->first;
    pcl::PointCloud<pcl::PointXYZI> model_centers = it->second;

    pcl::io::savePCDFileASCII(scene_name + "/debug/" + class_name + ".pcd", model_centers);

    Eigen::MatrixXf grid;
    voteToGrid(model_centers, grid, min_bound, max_bound, cell_size);
    saveGridToPGMFile(scene_name + "/debug/" + class_name + ".pgm", grid);

    pcl::PointCloud<pcl::PointXYZ> local_maxima;
    findLocalMaxima(grid, window_size, min_bound, cell_size, local_maxima_threshold, local_maxima);

    pcl::io::savePCDFileASCII(scene_name + "/debug/" + class_name + "_local_max.pcd", local_maxima);

    if (use_icp)
    {

      std::vector<pcl::PointCloud<pcl::PointXYZ> > result, no_intersection_result;
      std::vector<float> score;

      refineWithICP(class_to_full_pointcloud[class_name], local_maxima, scene, num_rotations_icp, icp_threshold,
                    result, score);

      if (result.size() > 0)
        removeIntersecting(result, score, no_intersection_result);

      for (size_t i = 0; i < no_intersection_result.size(); i++)
      {
        std::stringstream ss;
        ss << scene_name << "/result/" << class_name << "_" << i << ".pcd";
        pcl::io::savePCDFileASCII(ss.str(), no_intersection_result[i]);
      }

    }

  }

  pcl::io::savePCDFileASCII(scene_name + "/result/scene.pcd", *scene);

  return 0;
}

