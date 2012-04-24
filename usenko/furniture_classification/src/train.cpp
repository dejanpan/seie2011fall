/*
 * scan.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: vsu
 */

#include <iostream>
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

const int featureLength = pcl::SGF4_SIZE;
typedef pcl::Histogram<featureLength> featureType;
typedef pcl::SGF4Estimation<pcl::PointXYZ, featureType> featureEstimation;

std::vector<featureType> features;
std::vector<Eigen::Vector4f> centroids;
std::vector<std::string> name;

void filter_segments(const std::vector<std::vector<int> > & segment_indices,
                     std::vector<std::vector<int> > & new_segment_indices, size_t min_points_in_segment)
{
  new_segment_indices.clear();
  for (size_t i = 0; i < segment_indices.size(); i++)
  {
    if (segment_indices[i].size() > min_points_in_segment)
    {
      new_segment_indices.push_back(segment_indices[i]);
    }
  }
}

cv::Mat transform_to_mat(std::vector<featureType> features)
{
  cv::Mat res(features.size(), featureLength, CV_32F);
  for (size_t i = 0; i < features.size(); i++)
  {
    for (int j = 0; j < featureLength; j++)
    {
      res.at<float>(i,j) = features[i].histogram[j];
    }
  }

  return res;
}

int main(int argc, char** argv)
{
  size_t min_points_in_segment = 100;
  int num_clusters = 5;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile("data/scans/Aluminium_Group_EA_119_0000F152-centered/rotation0_distance4_tilt-15_shift0.pcd",
                       cloud);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setComputeNormals(true);

  // Set parameters
  mls.setInputCloud(cloud.makeShared());
  mls.setPolynomialFit(true);
  mls.setPolynomialOrder(2);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);

  // Reconstruct
  mls.process(mls_points);

  pcl::PointCloud<pcl::PointXYZ> mls_cloud;
  pcl::PointCloud<pcl::Normal> mls_normal;
  pcl::copyPointCloud(mls_points, mls_cloud);
  pcl::copyPointCloud(mls_points, mls_normal);

  std::vector<std::vector<int> > segment_indices;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  pcl::RegionGrowing<pcl::PointXYZ> region_growing;
  region_growing.setCloud(mls_cloud.makeShared());
  region_growing.setNormals(mls_normal.makeShared());
  region_growing.setNeighbourSearchMethod(tree);
  region_growing.setResidualTest(true);
  region_growing.setResidualThreshold(0.05);
  region_growing.setCurvatureTest(false);
  region_growing.setSmoothMode(false);
  region_growing.setSmoothnessThreshold(40 * M_PI / 180);
  region_growing.segmentPoints();
  segment_indices = region_growing.getSegments();
  colored_cloud = region_growing.getColoredCloud();

  //pcl::visualization::PCLVisualizer viz;
  //viz.initCameraParameters();
  //viz.addCoordinateSystem(0.1);


  pcl::PointCloud<featureType> feature;
  featureEstimation feature_estimator;
  feature_estimator.setInputCloud(mls_cloud.makeShared());
  feature_estimator.setSearchMethod(tree);
  feature_estimator.setKSearch(10);

  std::vector<std::vector<int> > new_segment_indices;
  filter_segments(segment_indices, new_segment_indices, min_points_in_segment);

  for (size_t i = 0; i < new_segment_indices.size(); i++)
  {
    // Compute feature vector for segment
    boost::shared_ptr<std::vector<int> > idx(new std::vector<int>());
    *idx = new_segment_indices[i];
    feature_estimator.setIndices(idx);
    feature_estimator.compute(feature);

    // Compute centroid of segment
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(mls_cloud, new_segment_indices[i], centroid);
    // Assuming that center of the model is at (0,0,0)
    // we get the vector from segment centroid to model center.
    centroid *= -1;

    features.push_back(feature.points[0]);
    centroids.push_back(centroid);
    name.push_back("chair");

    int num_elements = sizeof(feature.points[0].histogram) / sizeof(float);
    std::cout << "Feature " << i << "[";
    for (int j = 0; j < num_elements; j++)
    {
      std::cout << feature.points[0].histogram[j] << " ";
    }
    std::cout << "]\n";

  }

  cv::Mat feature_vectors = transform_to_mat(features);
  cv::Mat centers(num_clusters, featureLength, feature_vectors.type()), labels;



  cv::kmeans(feature_vectors, num_clusters, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 3,
             cv::KMEANS_RANDOM_CENTERS, centers);

  std::cout << centers.cols << " " << centers.rows << " " << centers.dims << "\n";

  for (int i = 0; i < centers.rows; i++)
  {
    std::cout << "Centroid " << i << "[";
    for (int j = 0; j < centers.cols; j++)
    {
      std::cout << centers.at<float> (i, j) << " ";
    }
    std::cout << "]\n";
  }

  for (int i = 0; i < labels.rows; i++)
  {
    std::cout << "Feature " << i << " goes to Centroid " << labels.at<int> (i) << "\n";
  }

  //viz.addPointCloud(colored_cloud);

  //viz.spin();


  return 0;
}
