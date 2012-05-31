/*
 * classify.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: vsu
 */

#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/passthrough.h>
#include <sac_3dof.h>
#include <training.h>
#include <set>


template<class ScenePoint>
void findVotedSegments(const pcl::PointCloud<pcl::PointXYZ> & local_maxima_cloud, float cell_size, boost::shared_ptr<pcl::PointCloud<
    ScenePoint> > scene, const std::vector<std::vector<int> > & segment_indices, const pcl::PointCloud<
    pcl::PointXYZI> & votes, const std::vector<int> & vote_segment_idx,
                       std::vector<pcl::PointCloud<ScenePoint> > & voted_segments)
{

  std::vector<std::set<int> > segment_combinations;

  for (size_t j = 0; j < local_maxima_cloud.points.size(); j++)
  {
    pcl::PointXYZ local_maxima = local_maxima_cloud.points[j];
    std::vector<int> idx;

    for (size_t i = 0; i < votes.points.size(); i++)
    {

      bool in_cell_x1 = votes.points[i].x > (local_maxima.x - 5 * cell_size);
      bool in_cell_x2 = votes.points[i].x < (local_maxima.x + 5 * cell_size);
      bool in_cell_y1 = votes.points[i].y > (local_maxima.y - 5 * cell_size);
      bool in_cell_y2 = votes.points[i].y < (local_maxima.y + 5 * cell_size);

      if (in_cell_x1 && in_cell_x2 && in_cell_y1 && in_cell_y2)
      {
        idx.push_back(i);
      }
    }

    //std::cerr << "Number of points voted " << idx.size() << std::endl;

    std::set<int> segment_idx;

    for (size_t i = 0; i < idx.size(); i++)
    {
      segment_idx.insert(vote_segment_idx[idx[i]]);
    }

    //std::cerr << "Number of segments voted " << segment_idx.size() << std::endl;
    segment_combinations.push_back(segment_idx);

    //    for (std::set<int>::iterator it = segment_idx.begin(); it != segment_idx.end(); it++)
    //    {
    //      pcl::PointCloud<pcl::PointXYZ> segment(*scene, segment_indices[*it]);
    //      voted_segments += segment;
    //    }

  }

  std::unique(segment_combinations.begin(), segment_combinations.end());

  for (size_t i = 0; i < segment_combinations.size(); i++)
  {
    pcl::PointCloud<ScenePoint> cloud;

    for (std::set<int>::iterator it = segment_combinations[i].begin(); it != segment_combinations[i].end(); it++)
    {
      pcl::PointCloud<ScenePoint> segment(*scene, segment_indices[*it]);
      cloud += segment;
    }

    voted_segments.push_back(cloud);

  }

}



template<class ScenePoint>
void refineWithSegmentsICP(const std::vector<std::string> & models,
                           const std::vector<pcl::PointCloud<ScenePoint> > & segments_vector, std::vector<
                               pcl::PointCloud<pcl::PointXYZ> > & result, std::vector<float> & score)
{

  for (size_t model_idx = 0; model_idx < models.size(); model_idx++)
  {
    pcl::PointCloud<pcl::PointNormal>::Ptr full_model(new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile(models[model_idx], *full_model);

    //pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ> sac;

    for (size_t i = 0; i < segments_vector.size(); i++)
    {

      pcl::SampleConsensusModel3DOF<pcl::PointNormal>::Ptr
                                                           model(
                                                                 new pcl::SampleConsensusModel3DOF<pcl::PointNormal>(
                                                                                                                     full_model));

      //pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelRegistration<
      //    pcl::PointXYZ>(full_model));

      // TODO dont use makeShared
      model->setTarget(segments_vector[i].makeShared());
      pcl::RandomSampleConsensus<pcl::PointNormal> ransac(model);

      ransac.setDistanceThreshold(.01);
      ransac.
      ransac.computeModel();

      Eigen::VectorXf model_coefs;
      ransac.getModelCoefficients(model_coefs);
      std::cerr << "Model coefs " << model_coefs << std::endl;


      pcl::visualization::PCLVisualizer viz;
      viz.initCameraParameters();
      viz.updateCamera();
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);

      viz.addPointCloud<pcl::PointXYZ> (scene, "cloud2");
      viz.addPointCloud<pcl::PointXYZ> (segments_vector[i].makeShared());//, single_color);
      viz.spin();

    }
  }
}

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
  pcl::PointCloud<pcl::PointNormal>::Ptr scene(new pcl::PointCloud<pcl::PointNormal>);
  std::vector<std::vector<int> > segment_indices;

  append_segments_from_file(scene_file_name, features, centroids, classes, min_points_in_segment, *scene,
                            segment_indices, &min_bound, &max_bound);

  featureType min, max;
  normalizeFeatures(features, min, max, min_train, max_train);

  classes.clear();

  std::cout << "Found segments " << features.size() << " Cluster size " << feature_cloud->points.size() << std::endl;

  pcl::search::KdTree<featureType> feature_search;
  feature_search.setInputCloud(feature_cloud);

  std::map<std::string, pcl::PointCloud<pcl::PointXYZI> > votes;
  std::map<std::string, std::vector<int> > vote_segment_idx;

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
        for (size_t k = 0; k < model_centers_transformed_weighted.size(); k++)
        {
          model_centers_transformed_weighted.points[k].intensity = (1.0 / distances[j]) * (1.0 / model_centers.size());
          vote_segment_idx[class_name].push_back(i);
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

      //refineWithICP(class_to_full_pointcloud[class_name], local_maxima, scene, num_rotations_icp, icp_threshold,
      //              result, score);

      std::cerr << class_name << std::endl;
      std::vector<pcl::PointCloud<pcl::PointNormal> > segments_vector;
      findVotedSegments<pcl::PointNormal>(local_maxima, cell_size, scene, segment_indices, votes[class_name],
                        vote_segment_idx[class_name], segments_vector);
      refineWithSegmentsICP<pcl::PointNormal>(class_to_full_pointcloud[class_name], segments_vector, result, score);

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

