/*
 * classify.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: vsu
 */

#include <training.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <fstream>

bool intersectXY(const pcl::PointCloud<pcl::PointXYZ> & cloud1, const pcl::PointCloud<pcl::PointXYZ> & cloud2)
{

  pcl::PointXYZ min1, max1, min2, max2;
  pcl::getMinMax3D<pcl::PointXYZ>(cloud1, min1, max1);
  pcl::getMinMax3D<pcl::PointXYZ>(cloud2, min2, max2);

  bool intersectX, intersectY;
  if (min1.x < min2.x)
    intersectX = max1.x > min2.x;
  else
    intersectX = max2.x > min1.x;

  if (min1.y < min2.y)
    intersectY = max1.y > min2.y;
  else
    intersectY = max2.y > min1.y;

  return intersectX && intersectY;

}

void removeIntersecting(const std::vector<pcl::PointCloud<pcl::PointXYZ> > & result, const std::vector<float> & score,
                        std::vector<pcl::PointCloud<pcl::PointXYZ> > & no_intersect_result)
{
  for (size_t i = 0; i < result.size(); i++)
  {
    bool best = true;
    for (size_t j = 0; j < result.size(); j++)
    {
      if (intersectXY(result[i], result[j]))
      {
      if (score[i] > score[j])
        best = false;
      }

    }
    if (best)
    {
      no_intersect_result.push_back(result[i]);
    }
  }

}

void refineWithICP(const std::vector<std::string> & models, const pcl::PointCloud<pcl::PointXYZ> & local_maxima,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr scene, int num_rotations_icp, float icp_threshold, std::vector<
                       pcl::PointCloud<pcl::PointXYZ> > & result, std::vector<float> & score)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  for (size_t model_idx = 0; model_idx < models.size(); model_idx++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr full_model(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(models[model_idx], *full_model);

    std::cerr << "Checking model:" << models[model_idx] << std::endl;

    for (size_t i = 0; i < local_maxima.points.size(); i++)
    {
      for (int j = 0; j < num_rotations_icp; j++)
      {
        float angle = 2 * M_PI * j / num_rotations_icp;
        Eigen::AngleAxis<float> rot(angle, Eigen::Vector3f(0, 0, 1));

        Eigen::Affine3f transform;
        transform.setIdentity();
        transform.translate(local_maxima.points[i].getVector3fMap());
        transform.rotate(rot);

        pcl::PointCloud<pcl::PointXYZ>::Ptr full_model_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*full_model, *full_model_transformed, transform);

        icp.setInputCloud(full_model_transformed);
        icp.setInputTarget(scene);
        pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*Final);
        std::cerr << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

        if (icp.getFitnessScore() < icp_threshold)
        {
          result.push_back(*Final);
          score.push_back(icp.getFitnessScore());

          //              pcl::visualization::PCLVisualizer viz;
          //              viz.initCameraParameters();
          //              viz.updateCamera();
          //              pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(Final, 0, 255, 0);
          //              viz.addPointCloud<pcl::PointXYZ> (Final, single_color);
          //              viz.addPointCloud<pcl::PointXYZ> (scene, "cloud2");
          //              viz.spin();
        }

      }

    }
  }
}

void voteToGrid(pcl::PointCloud<pcl::PointXYZI> & model_centers, Eigen::MatrixXf & grid,
                const pcl::PointXYZ & min_bound, const pcl::PointXYZ & max_bound, float cell_size)
{

  int image_x_width = (int)((max_bound.x - min_bound.x) / cell_size);
  int image_y_width = (int)((max_bound.y - min_bound.y) / cell_size);

  grid = Eigen::MatrixXf::Zero(image_x_width, image_y_width);

  for (size_t i = 0; i < model_centers.points.size(); i++)
  {
    int vote_x = (model_centers.points[i].x - min_bound.x) / cell_size;
    int vote_y = (model_centers.points[i].y - min_bound.y) / cell_size;
    if ((vote_x >= 0) && (vote_y >= 0) && (vote_x < image_x_width) && (vote_y < image_y_width))
      grid(vote_x, vote_y) += model_centers.points[i].intensity;
  }
}

void saveGridToPGMFile(const std::string & filename, const Eigen::MatrixXf & grid)
{
  Eigen::MatrixXi img = (grid * 255.0 / grid.maxCoeff()).cast<int> ();

  std::ofstream f(filename.c_str());
  f << "P2\n" << grid.cols() << " " << grid.rows() << "\n255\n";
  f << img;
}

void findLocalMaxima(const Eigen::MatrixXf & grid, const float window_size, const pcl::PointXYZ & min_bound,
                     const float cell_size, const float local_maxima_threshold,
                     pcl::PointCloud<pcl::PointXYZ> & local_maxima)
{

  float max, min;
  max = grid.maxCoeff();
  min = grid.minCoeff();

  float threshold = min + (max - min) * local_maxima_threshold;

  int window_size_pixels = window_size / cell_size;

  // Make window_size_pixels even
  if (window_size_pixels % 2 == 0)
    window_size_pixels++;

  int side = window_size_pixels / 2;

  for (int i = side; i < (grid.rows() - side); i++)
  {
    for (int j = side; j < (grid.cols() - side); j++)
    {

      float max;
      Eigen::MatrixXf window = grid.block(i - side, j - side, window_size_pixels, window_size_pixels);
      max = window.maxCoeff();

      assert(window.cols() == window_size_pixels);
      assert(window.rows() == window_size_pixels);

      // if max of the window is in its center then this point is local maxima
      if ((max == grid(i, j)) && (max > 0) && (max > threshold))
      {
        pcl::PointXYZ point;
        point.x = i * cell_size + min_bound.x;
        point.y = j * cell_size + min_bound.y;
        point.z = 0;
        local_maxima.points.push_back(point);
      }
    }
  }

  local_maxima.width = local_maxima.points.size();
  local_maxima.height = 1;
  local_maxima.is_dense = true;

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

    pcl::io::savePCDFileASCII(class_name + ".pcd", model_centers);

    Eigen::MatrixXf grid;
    voteToGrid(model_centers, grid, min_bound, max_bound, cell_size);
    saveGridToPGMFile(class_name + ".pgm", grid);

    pcl::PointCloud<pcl::PointXYZ> local_maxima;
    findLocalMaxima(grid, window_size, min_bound, cell_size, local_maxima_threshold, local_maxima);

    pcl::io::savePCDFileASCII(class_name + "_local_max.pcd", local_maxima);

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
        ss << "Result_" << class_name << "_" << i << ".pcd";
        pcl::io::savePCDFileASCII(ss.str(), no_intersection_result[i]);
      }

    }

  }

  return 0;
}

