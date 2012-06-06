/*
 * PrimitivesExtract.h
 *
 *  Created on: Jun 4, 2012
 *      Author: Karol Hausman
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/boundary.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/keypoints/harris_keypoint3D.h>

#ifndef PRIMITIVESEXTRACT_H_
#define PRIMITIVESEXTRACT_H_

template<typename PointType>
class PrimitivesExtract {
public:
	typedef pcl::PointXYZRGBA RefPointType;
	typedef pcl::PointCloud<PointType> Cloud;
	typedef pcl::PointCloud<RefPointType> RefCloud;
	typedef typename RefCloud::Ptr RefCloudPtr;
	typedef typename RefCloud::ConstPtr RefCloudConstPtr;
	typedef typename Cloud::Ptr CloudPtr;
	typedef typename Cloud::ConstPtr CloudConstPtr;
	typedef typename pcl::search::KdTree<PointType> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;

	PrimitivesExtract() {
		find_boundaries_radius_search_ = 0.03;
		best_curv_percent_ = 0.2;
		best_intens_percent_ = 0.07;
		extract_neigh_radius_ = 0.05;
		radius_harris_ = 0.01;
		radius_search_harris_ = 0.01;
		plane_distance_treshold_ = 0.005;
		min_plane_inliers_ = 70;
		line_distance_tresh_ = 0.002;
		min_line_inliers_ = 50;
		eliminate_line_neigh_radius_ = 0.008; //0.002
		convex_corners_only_ = true;
	}

	PrimitivesExtract(CloudPtr cloud) {
		find_boundaries_radius_search_ = 0.03;
		best_curv_percent_ = 0.17;
		best_intens_percent_ = 0.07;
		extract_neigh_radius_ = 0.05;
		cloud_ = cloud;
		radius_harris_ = 0.01;
		radius_search_harris_ = 0.01;
		plane_distance_treshold_ = 0.005;
		min_plane_inliers_ = 70;
		line_distance_tresh_ = 0.002;
		min_line_inliers_ = 50;
		eliminate_line_neigh_radius_ = 0.008; //0.002
		convex_corners_only_ = true;

	}

	void findBoundaries(const CloudConstPtr cloud, Cloud &result);

	bool extractCornerVector(CloudConstPtr input, std::vector<CloudPtr>& result,
			int number = 0);
	bool extractLines(const CloudConstPtr &cloud,
			std::vector<CloudPtr> &result_vector,
			std::vector <pcl::ModelCoefficients::Ptr> &coefficients_vector, int lines_number = 0);
	void removePrimitive(const CloudConstPtr &cloud,
			pcl::PointIndices::Ptr &indices_to_remove, Cloud &result);
	bool extractCorners(const CloudConstPtr cloud, Cloud &result,
			Cloud &result_debug, int number = 0);

private:

	void extractNeighbor(const CloudConstPtr cloud, PointType &searchPoint,
			pcl::PointIndices::Ptr &inliers, int& size, float radius = 0);
	int countPlanes(const CloudConstPtr cloud);
	bool extractPlane(const CloudConstPtr cloud,
			pcl::PointIndices::Ptr &inliers);

	float find_boundaries_radius_search_;
	float best_curv_percent_;
	float best_intens_percent_;
	float extract_neigh_radius_;
	float radius_harris_;
	float radius_search_harris_;
	float plane_distance_treshold_;
	float min_plane_inliers_;
	float line_distance_tresh_;
	float min_line_inliers_;
	float eliminate_line_neigh_radius_;
	bool convex_corners_only_;
	CloudPtr cloud_;
};

#endif /* PRIMITIVESEXTRACT_H_ */
