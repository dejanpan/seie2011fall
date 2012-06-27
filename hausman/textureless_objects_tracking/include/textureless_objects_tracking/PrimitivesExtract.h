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
//
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "cv_bridge/CvBridge.h"

#include <textureless_objects_tracking/cornerFind.h>
#include <object_part_decomposition/ClassifyScene.h>
#include <object_part_decomposition/point_type.h>

#include <ros/ros.h>


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
		//parameters for lines extraction
		find_normals_radius_search_ = 0.03;
		best_curv_percent_ = 0.17;
		line_distance_tresh_ = 0.002;
		min_line_inliers_ = 50;
		eliminate_line_neigh_radius_ = 0.008; //0.002
		shrink_line_percent_ = 1;
		euclidian_clustering_after_line_projection_ = false;
		if (euclidian_clustering_after_line_projection_) {
			euclidian_line_cluster_tolerance_ = 0.005;
			euclidian_min_cluster_size_ = 30;
			euclidian_max_cluster_size_ = 25000;

		}
		//parameters in general
		extract_neigh_radius_ = 0.05;
		plane_distance_treshold_ = 0.005;
		min_plane_inliers_ = 70;

		//parameters for cylinder extraction

		//parameters for 3d circle extraction

		//parameters for corners extraction
		best_intens_percent_ = 0.07;
		radius_harris_ = 0.01;
		radius_search_harris_ = 0.01;
		convex_corners_only_ = true;
	}

	PrimitivesExtract(CloudPtr cloud) {

		_segmentation_srv =_nh.serviceClient<object_part_decomposition::ClassifyScene>("/classify_scene/classify_scene");
		_corner_finder = _nh.serviceClient<textureless_objects_tracking::cornerFind>("find_corners");
		if (!_corner_finder.waitForExistence(ros::Duration(5.0))){
			ROS_ERROR("find_corners service not found");
			exit(1);
		}
		//parameters for lines extraction
		best_curv_percent_ = 0.17;
		line_distance_tresh_ = 0.002;
		min_line_inliers_ = 50;
		eliminate_line_neigh_radius_ = 0.01; //0.002
		shrink_line_percent_ = 1;
		euclidian_clustering_after_line_projection_ = true;
		if (euclidian_clustering_after_line_projection_) {
			euclidian_line_cluster_tolerance_ = 0.005;
		}
		//parameters in general
		cloud_ = cloud;
		extract_neigh_radius_ = 0.05;
		plane_distance_treshold_ = 0.005;
		min_plane_inliers_ = 70;
		find_normals_radius_search_ = 0.03;
		euclidian_min_cluster_size_ = 30;
		euclidian_max_cluster_size_ = 25000;

		euclidian_clustering_after_circular_extraction_ = true;
		//parameters for cylinder extraction
		cylinder_distance_treshold_ = 0.03;
		cylinder_min_radius_ = 0.03;
		cylinder_max_radius_ = 0.15;
		cylinder_min_inliers_ = 1000;
		if (euclidian_clustering_after_circular_extraction_)
			euclidian_cylinder_cluster_tolerance_ = 0.03;

		//parameters for 3d circle extraction
		circle_distance_treshold_ = 0.003;
		circle_min_radius_ = 0.01;
		circle_max_radius_ = 0.15;
		circle_min_inliers_ = 270;
		if (euclidian_clustering_after_circular_extraction_)
			euclidian_circle_cluster_tolerance_ = 0.03;

		//parameters for corners extraction
		best_intens_percent_ = 0.07;
		radius_harris_ = 0.01;
		radius_search_harris_ = 0.01;
		convex_corners_only_ = true;

	}

	bool extractCornerVector(CloudConstPtr input, std::vector<CloudPtr>& result,
			int number = 0);
	bool extractLineVector(const CloudConstPtr& input,
			std::vector<CloudPtr>& result,
			std::vector<Eigen::Vector3f> &directions_vector,
			int lines_number = 0);
	bool extractCylinderVector(
			const CloudConstPtr &cloud, std::vector<CloudPtr> &result,
			int cylinders_number=0);
	bool extractCircleVector(
			const CloudConstPtr &cloud, std::vector<CloudPtr> &result,
			int cylinders_number=0);
	bool getCornersToPush(cv::Mat& topview, textureless_objects_tracking::cornerFind::Response& res);
	bool getSegments(const CloudConstPtr cloud,  pcl::PointCloud<pcl::PointXYZLRegion>::Ptr &cloud_out);

private:
	void computeNormals(const CloudConstPtr cloud,
			pcl::PointCloud<pcl::Normal>::Ptr normals_cloud);
	void findBoundaries(const CloudConstPtr cloud, Cloud &result);
	bool extractLines(const CloudConstPtr &cloud,
			std::vector<CloudPtr> &result_vector,
			std::vector<pcl::ModelCoefficients::Ptr> &coefficients_vector,
			int lines_number = 0);
	bool extractCircular(const CloudConstPtr &cloud,
			std::vector<CloudPtr> &result, int cylinders_number = 0,std::string what="cylinder");
	void removePrimitive(const CloudConstPtr &cloud,
			pcl::PointIndices::Ptr &indices_to_remove, Cloud &result);
	void removePointsAroundLine(const CloudConstPtr &cloud, Cloud &result,
			Cloud &line, pcl::ModelCoefficients::Ptr &coefficients);
	bool extractCorners(const CloudConstPtr cloud, Cloud &result,
			Cloud &result_debug, int number = 0);
	void extractNeighbor(const CloudConstPtr cloud, PointType &searchPoint,
			pcl::PointIndices::Ptr &inliers, int& size, float radius = 0);
	int countPlanes(const CloudConstPtr cloud);
	bool extractPlane(const CloudConstPtr cloud,
			pcl::PointIndices::Ptr &inliers);
	void euclidianClustering(CloudPtr& cloudForEuclidianDistance,
			std::vector<pcl::PointIndices>& cluster_indices,
			float euclidian_cluster_tolerance);
	void lineDirectionPCA(const CloudConstPtr &cloud, Eigen::Vector3f &direction);

	float find_normals_radius_search_;
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
	float shrink_line_percent_;
	float euclidian_line_cluster_tolerance_;
	float euclidian_min_cluster_size_;
	float euclidian_max_cluster_size_;
	float cylinder_distance_treshold_;
	float cylinder_min_radius_;
	float cylinder_max_radius_;
	float cylinder_min_inliers_;
	float euclidian_cylinder_cluster_tolerance_;
	float circle_distance_treshold_;
	float circle_min_radius_;
	float circle_max_radius_;
	float circle_min_inliers_;
	float euclidian_circle_cluster_tolerance_;
	bool convex_corners_only_;
	bool euclidian_clustering_after_line_projection_;
	bool euclidian_clustering_after_circular_extraction_;
	ros::ServiceClient _corner_finder;
	ros::ServiceClient _segmentation_srv;
	ros::NodeHandle _nh;
	CloudPtr cloud_;
};

#endif /* PRIMITIVESEXTRACT_H_ */
