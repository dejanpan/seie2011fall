/*
 * PushPointEstimation.h
 *
 *  Created on: Jul 16, 2012
 *      Author: Karol Hausman
 */

#ifndef PUSHPOINTESTIMATION_H_
#define PUSHPOINTESTIMATION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <object_part_decomposition/point_type.h>
#include "textureless_objects_tracking/point_type.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/search/pcl_search.h>
#include <pcl/search/impl/organized.hpp>

#include <boost/math/special_functions/factorials.hpp>


class PushPointEstimation {
public:
	PushPointEstimation();
	virtual ~PushPointEstimation();
	void getPushCloud(pcl::PointCloud<pcl::PointXYZLRegion>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZLRegion>& push_point_cloud,pcl::PointCloud<pcl::PointXYZLRegion> &push_point);
	void getUncertainRegion(std::vector<pcl::PointCloud<pcl::PointXYZLRegion> >& cluster_pointclouds,pcl::PointCloud<pcl::PointXYZLRegion> &uncertain_cloud,std::string &what,double &min_probability);
	void getLabels(pcl::PointCloud<pcl::PointXYZLRegion>::Ptr &cloud);
	void euclidianClustering(
			pcl::PointCloud<pcl::PointXYZLRegion>::Ptr& cloudForEuclidianDistance,
			std::vector<pcl::PointCloud<pcl::PointXYZLRegion> >& cluster_pointclouds);
	void setAll3DCornersFromService(pcl::PointCloud<pcl::PointXYZLRegion>& concave_cloud,pcl::PointCloud<pcl::PointXYZLRegion>& convex_cloud);
	void getPushPoint(const pcl::PointCloud<pcl::PointXYZLRegion>::Ptr& push_cloud,pcl::PointCloud<pcl::PointXYZLRegion>& push_point_cloud);


	pcl::PointCloud<pcl::PointXYZLRegion> concave_cloud_;
	pcl::PointCloud<pcl::PointXYZLRegion> convex_cloud_;
    pcl::PointCloud<pcl::PointXYZLRegion>::Ptr label_rectangular_;
    pcl::PointCloud<pcl::PointXYZLRegion>::Ptr label_circular_;
    pcl::PointCloud<pcl::PointXYZLRegion>::Ptr label_other_;
    float euclidian_min_cluster_size_;
    float euclidian_max_cluster_size_;
    float euclidian_cluster_tolerance_;
    float max_distance_from_corner_concave_;
};

#endif /* PUSHPOINTESTIMATION_H_ */
