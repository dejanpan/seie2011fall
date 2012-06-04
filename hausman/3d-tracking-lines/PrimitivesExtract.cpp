/*
 * PrimitivesExtract.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: Karol Hausman
 */

#include "PrimitivesExtract.h"

inline bool comparison_intens(const pcl::PointXYZI& i,
		const pcl::PointXYZI& j) {
	return (i.intensity > j.intensity);
}

inline bool comparison_curvature(pcl::Normal i, pcl::Normal j) {
	return (i.curvature > j.curvature);
}

template<typename PointType> void PrimitivesExtract<PointType>::findBoundaries(
		const CloudConstPtr cloud, Cloud &result) {

	pcl::PointCloud<pcl::Normal>::Ptr temp_normals(
			new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_cloud(
			new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointType, pcl::Normal> norm_est;
	norm_est.setSearchMethod(KdTreePtr(new KdTree));
	norm_est.setInputCloud(cloud);
	norm_est.setRadiusSearch(find_boundaries_radius_search_);
	norm_est.compute(*normals_cloud);

	pcl::copyPointCloud(*normals_cloud, *temp_normals);

	std::sort(temp_normals->points.begin(), temp_normals->points.end(),
			comparison_curvature);
	int place = (int) (temp_normals->points.size() * best_curv_percent_);
	pcl::Normal tresh_point = *(temp_normals->points.begin() + place);
	float treshold = tresh_point.curvature;

	for (size_t i = 0; i < normals_cloud->size(); ++i) {
		if (normals_cloud->points[i].curvature > treshold)
			result.push_back(cloud->points.at(i));
	}

	result.width = result.points.size();
	result.height = 1;
	result.is_dense = false;

}

template<typename PointType> bool PrimitivesExtract<PointType>::extractCornerVector(
		CloudConstPtr cloud_input, std::vector<CloudPtr>& result, int number) {

	CloudPtr corners(new Cloud);
	CloudPtr corners_debug(new Cloud);

	extractCorners(cloud_input, *corners, *corners_debug, number);

	for (size_t j = 0; j < corners->points.size(); j++) {
		CloudPtr augmented_corner(new Cloud);
		CloudPtr augmented_corner2(new Cloud);

		augmented_corner2->push_back(corners->points[j]);
		extractNeighbor(cloud_, *augmented_corner2, *augmented_corner);
		if ((countPlanes(augmented_corner) == 4)
				|| (countPlanes(augmented_corner) == 3))
			result.push_back(augmented_corner);
	}
	if (result.size() == 0)
		return false;
	else
		return true;

}

template<typename PointType> void PrimitivesExtract<PointType>::extractNeighbor(
		const CloudConstPtr cloud, Cloud &searchCloud, Cloud &result) {

	pcl::KdTreeFLANN<PointType> kdtree;

	kdtree.setInputCloud(cloud);

	PointType searchPoint;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	for (size_t i = 0; i < searchCloud.size(); ++i) {
		searchPoint = searchCloud.points.at(i);

		result.push_back(searchPoint);

		if (kdtree.radiusSearch(searchPoint, extract_neigh_radius_,
				pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)

				result.push_back(cloud->points[pointIdxRadiusSearch[i]]);

		}

	}

	CloudPtr resultPtr(new Cloud(result));

	pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud(resultPtr);
	sor.setLeafSize(0.001f, 0.001f, 0.001f);
	sor.filter(result);

	result.width = result.points.size();
	result.height = 1;
	result.is_dense = false;

}

template<typename PointType> bool PrimitivesExtract<PointType>::extractPlane(
		const CloudConstPtr cloud, pcl::PointIndices::Ptr &inliers) {

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	if (cloud->size() == 0)
		return false;

	pcl::SACSegmentation<PointType> seg;
	// Optional
	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(plane_distance_treshold_);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() < min_plane_inliers_) {
		PCL_ERROR( "Could not estimate a plane model for the given dataset.");
		return false;
	}

	return true;

}

template<typename PointType> int PrimitivesExtract<PointType>::countPlanes(
		const CloudConstPtr cloud) {
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	int number = 0;

	CloudPtr nonplane_cloud(new Cloud);
	pcl::copyPointCloud(*cloud, *nonplane_cloud);
	for (u_int i = 0; i < 6; i++) {

		inliers.reset(new pcl::PointIndices);

		if (extractPlane(nonplane_cloud, inliers)) {

			pcl::ExtractIndices<PointType> extract;
			extract.setInputCloud(nonplane_cloud);
			extract.setIndices(inliers);
			extract.setNegative(true);

			extract.filter(*nonplane_cloud);
			number++;
		} else
			return number;
	}

	return number;

}

template<typename PointType> bool PrimitivesExtract<PointType>::extractCorners(
		const CloudConstPtr cloud, Cloud &result, Cloud &result_debug,
		int number) {

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity(
			new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity_after_treshhold(
			new pcl::PointCloud<pcl::PointXYZI>);

	pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI>* harris3D =
			new pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI>(
					pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI>::HARRIS);
	harris3D->setNonMaxSupression(true);
//		 harris3D->setThreshold(0.0009);
//		 harris3D->setThreshold(0.00011);

	harris3D->setRadius(radius_harris_);
	harris3D->setRadiusSearch(radius_search_harris_);
	harris3D->setInputCloud(cloud);
	harris3D->setRefine(false);
	harris3D->setMethod(
			pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI>::HARRIS);
	harris3D->compute(*cloud_intensity);

	if (cloud_intensity->size() > 0) {
		std::sort(cloud_intensity->points.begin(),
				cloud_intensity->points.end(), comparison_intens);
		if (number == 0) {
			int place = (int) (cloud_intensity->points.size()
					* best_intens_percent_);
			pcl::PointXYZI tresh_point = *(cloud_intensity->points.begin()
					+ place);
			float treshold = tresh_point.intensity;

			for (size_t i = 0; i < cloud_intensity->size(); ++i) {
				if (cloud_intensity->points[i].intensity > treshold)
					cloud_intensity_after_treshhold->points.push_back(
							cloud_intensity->points.at(i));
			}
		} else {
			if (number > cloud_intensity->size()) {
				PCL_ERROR("Number of corners is bigger than found corners.");
				return false;
			} else {

				for (size_t i = 0; i < number; ++i) {
					cloud_intensity_after_treshhold->points.push_back(
							cloud_intensity->points[i]);
				}
			}
		}

		pcl::KdTreeFLANN<PointType> kdtree;

		kdtree.setInputCloud(cloud);
		pcl::copyPointCloud(*cloud_intensity_after_treshhold, result);
		pcl::copyPointCloud(result, result_debug);

		PointType searchPoint;

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		float radius = 0.05;
		std::vector<int> neighbor_num;

		for (size_t j = 0; j < result.points.size(); j++) {

			searchPoint = result.points[j];

			if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
					pointRadiusSquaredDistance) > 0) {
				neighbor_num.push_back((int) pointIdxRadiusSearch.size());

			}
		}
		double avg = 0;
		std::vector<int>::iterator it;
		for (it = neighbor_num.begin(); it != neighbor_num.end(); it++)
			avg += *it;
		avg /= neighbor_num.size();

		if (neighbor_num.size() == result.points.size()) {
			for (int n = 0; n < neighbor_num.size(); n++) {
				if (neighbor_num[n] > avg) {
					neighbor_num.erase(neighbor_num.begin() + n);
					result.points.erase(result.points.begin() + n);
				}
			}
		} else
			return false;

	} else {
		return false;
	}
	result.width = result.points.size();
	result.height = 1;
	result.is_dense = false;

	return true;

}

