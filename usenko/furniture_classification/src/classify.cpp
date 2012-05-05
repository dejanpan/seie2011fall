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
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {

	if (argc < 5) {
		PCL_INFO ("Usage %s -database_file_name /database.yaml -scene_file_name /scene.pcd [options]\n", argv[0]);
		PCL_INFO (" * where options are:\n"
				"         -num_neighbours <X>  : number of neighbours. Default : 1\n"
				"");
		return -1;
	}

	std::string database_file_name = "database.yaml";
	std::string scene_file_name;
	//"data/scans/Sideboards/Spatio_Sideboard_160_0000F1C5_squished-centered/rotation0_distance4_tilt-15_shift0.pcd";
	int min_points_in_segment = 300;
	int num_neighbours = 1;
	float cell_size = 0.01;

	pcl::console::parse_argument(argc, argv, "-database_file_name", database_file_name);
	pcl::console::parse_argument(argc, argv, "-scene_file_name", scene_file_name);
	pcl::console::parse_argument(argc, argv, "-num_neighbours", num_neighbours);

	databaseType database;
	pcl::PointCloud<featureType>::Ptr feature_cloud(new pcl::PointCloud<
			featureType>());
	featureType min_train, max_train;
	load_codebook(database_file_name, database, *feature_cloud, min_train,
			max_train);

	std::vector<featureType> features;
	pcl::PointCloud<pcl::PointXYZ> centroids;
	std::vector<std::string> classes;

	append_segments_from_file(scene_file_name, features, centroids, classes,
			min_points_in_segment);

	featureType min, max;
	normalizeFeatures(features, min, max, min_train, max_train);

	classes.clear();

	std::cout << "Found segments " << features.size() << " Cluster size "
			<< feature_cloud->points.size() << std::endl;

	pcl::search::KdTree<featureType> feature_search;
	feature_search.setInputCloud(feature_cloud);

	std::map<std::string, pcl::PointCloud<pcl::PointXYZI> > votes;

	for (size_t i = 0; i < features.size(); i++) {
		std::vector<int> indices;
		std::vector<float> distances;
		feature_search.nearestKSearch(features[i], num_neighbours, indices,
				distances);

		for (size_t j = 0; j < indices.size(); j++) {
			for (std::map<std::string, pcl::PointCloud<pcl::PointXYZ> >::const_iterator
					it = database[feature_cloud->at(indices[j])].begin(); it
					!= database[feature_cloud->at(indices[j])].end(); it++) {
				std::string class_name = it->first;
				pcl::PointCloud<pcl::PointXYZ> model_centers = it->second;
				pcl::PointCloud<pcl::PointXYZ> model_centers_transformed;
				pcl::PointCloud<pcl::PointXYZI>
						model_centers_transformed_weighted;

				Eigen::Affine3f transform;
				transform.setIdentity();
				transform.translate(centroids[i].getVector3fMap());
				pcl::transformPointCloud(model_centers,
						model_centers_transformed, transform);

				pcl::copyPointCloud(model_centers_transformed,
						model_centers_transformed_weighted);

				// TODO
				for (size_t i = 0; i
						< model_centers_transformed_weighted.size(); i++) {
					model_centers_transformed_weighted.points[i].intensity
							= (1.0 / distances[j]) * (1.0
									/ model_centers.size());
				}

				votes[class_name] += model_centers_transformed_weighted;
			}
		}

	}

	//	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr = votes["Chairs"].makeShared();
	//
	//	std::cout << ptr->size() << std::endl;
	//
	//	pcl::visualization::PCLVisualizer viz;
	//	viz.initCameraParameters();
	//	viz.updateCamera();
	//	viz.addPointCloud<pcl::PointXYZ>(ptr);
	//
	//	viz.spin();

	pcl::PointCloud<pcl::PointXYZ> scene;
	pcl::io::loadPCDFile(scene_file_name, scene);
	pcl::PointXYZ min_bound, max_bound;
	pcl::getMinMax3D<pcl::PointXYZ>(scene, min_bound, max_bound);
	int image_x_width = (int) ((max_bound.x - min_bound.x) / cell_size);
	int image_y_width = (int) ((max_bound.y - min_bound.y) / cell_size);
	std::cout << "Size " << image_x_width << "x" << image_y_width << std::endl;

	for (std::map<std::string, pcl::PointCloud<pcl::PointXYZI> >::const_iterator
			it = votes.begin(); it != votes.end(); it++) {
		std::string class_name = it->first;
		pcl::PointCloud<pcl::PointXYZI> model_centers = it->second;

		pcl::io::savePCDFileASCII(class_name + ".pcd", model_centers);

		cv::Mat image = cv::Mat::zeros(image_x_width, image_y_width, CV_32FC1);

		for (size_t i = 0; i < model_centers.points.size(); i++) {
			int vote_x = (model_centers.points[i].x - min_bound.x) / cell_size;
			int vote_y = (model_centers.points[i].y - min_bound.y) / cell_size;
			if ((vote_x > 0) && (vote_y > 0) && (vote_x < image.rows)
					&& (vote_x < image.cols) && (model_centers.points[i].z > 0))
				image.at<float> (vote_x, vote_y)
						+= model_centers.points[i].intensity;
		}

		cv::Mat image_normalized;
		cv::normalize(image, image_normalized, 0.0, 255.0, cv::NORM_MINMAX);

		cv::circle(image_normalized, cv::Point(-min_bound.x / cell_size,
				-min_bound.y / cell_size), 4, 255);

		//cv::Mat image_normalized_char;
		//image_normalized.convertTo(image_normalized_char, CV_8UC3);
		//image_normalized_char.at<char> (-min_bound.x / cell_size, -min_bound.y
		//		/ cell_size, 1) = 250;
		cv::imwrite(class_name + ".png", image_normalized);

	}

	return 0;
}

