/*
 * DenseReconstruction.h
 *
 *  Created on: Aug 17, 2012
 *      Author: Karol Hausman
 */

#ifndef DENSERECONSTRUCTION_H_
#define DENSERECONSTRUCTION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>


#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>

#include "textureless_objects_tracking/point_type.h"


class DenseReconstruction {
public:
	DenseReconstruction(pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr saved_cloud);
	virtual ~DenseReconstruction();
	void planeSegmentation(const pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud,
			pcl::ModelCoefficients &coefficients, pcl::PointIndices &inliers);
	void planeExtraction(const pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud_input,pcl::PointIndices::Ptr &inliers,pcl::PointCloud<pcl::PointXYZLRegionF> &cloud_output);
	void normalsEstimation(const pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud);
	void regionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &segments);
	void extractEuclideanClustersCurvature(std::vector<pcl::PointIndices::Ptr> &clusters);




	pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_operational_;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr region_grow_point_cloud_;
protected:
	pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;

};

#endif /* DENSERECONSTRUCTION_H_ */
