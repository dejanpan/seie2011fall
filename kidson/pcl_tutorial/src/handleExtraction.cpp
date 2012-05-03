#include <iostream>

// PCL stuff
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/common/common.h"
#include <pcl/io/pcd_io.h>
#include "pcl/segmentation/extract_clusters.h"
#include <pcl/features/normal_3d.h>
#include <pcl/common/angles.h>
#include "pcl/common/common.h"

#include "tf/tf.h"

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::search::KdTree<Point> KdTree;
typedef KdTree::Ptr KdTreePtr;
//typedef pcl::KdTree<Point>::Ptr KdTreePtr;
//typedef typename pcl::search::Search<Point>::Ptr KdTreePtr;

void extractHandles(PointCloudPtr& cloudInput, std::vector<pcl::PointIndices>& handle_clusters) {
	// PCL objects
	//pcl::PassThrough<Point> vgrid_;                   // Filtering + downsampling object
	pcl::VoxelGrid<Point> vgrid_; // Filtering + downsampling object
	pcl::NormalEstimation<Point, pcl::Normal> n3d_; //Normal estimation
	// The resultant estimated point cloud normals for \a cloud_filtered_
	pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;
	pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_; // Planar segmentation object
	pcl::SACSegmentation<Point> seg_line_; // Planar segmentation object
	pcl::ProjectInliers<Point> proj_; // Inlier projection object
	pcl::ExtractIndices<Point> extract_; // Extract (too) big tables
	pcl::ConvexHull<Point> chull_;
	pcl::ExtractPolygonalPrismData<Point> prism_;
	pcl::PointCloud<Point> cloud_objects_;
	pcl::EuclideanClusterExtraction<Point> cluster_, handle_cluster_;

	double object_cluster_tolerance_, handle_cluster_tolerance_,
			cluster_min_height_, cluster_max_height_, voxel_size_;
	int object_cluster_min_size_, object_cluster_max_size_,
			handle_cluster_min_size_, handle_cluster_max_size_;

	double sac_distance_, normal_distance_weight_, z_min_limit_, z_max_limit_;
	double y_min_limit_, y_max_limit_, x_min_limit_, x_max_limit_;
	double eps_angle_, seg_prob_;
	int k_, max_iter_, min_table_inliers_, nr_cluster_;

	sac_distance_ = 0.05;  //0.02
	normal_distance_weight_ = 0.05;
	max_iter_ = 500;
	eps_angle_ = 30.0; //20.0
	seg_prob_ = 0.99;
	btVector3 axis(0.0, 0.0, 1.0);

	seg_.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
	seg_.setMethodType(pcl::SAC_RANSAC);
	seg_.setDistanceThreshold(sac_distance_);
	seg_.setNormalDistanceWeight(normal_distance_weight_);
	seg_.setOptimizeCoefficients(true);
	seg_.setAxis(Eigen::Vector3f(fabs(axis.getX()), fabs(axis.getY()), fabs(axis.getZ())));
	seg_.setEpsAngle(pcl::deg2rad(eps_angle_));
	seg_.setMaxIterations(max_iter_);
	seg_.setProbability(seg_prob_);

	object_cluster_tolerance_ = 0.03;
	object_cluster_min_size_ = 200;
	cluster_.setClusterTolerance(object_cluster_tolerance_);
	//cluster_.setSpatialLocator(0);
	cluster_.setMinClusterSize(object_cluster_min_size_);
	//clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> >();
	KdTreePtr clusters_tree_(new KdTree);
	clusters_tree_->setEpsilon(1);
	cluster_.setSearchMethod(clusters_tree_);

	nr_cluster_ = 1;

	cluster_min_height_ = 0.03;
	cluster_max_height_ = 0.1;

	handle_cluster_tolerance_ = 0.03; //0.02
	handle_cluster_min_size_ = 40;
	handle_cluster_max_size_ = 500;
	handle_cluster_.setClusterTolerance(handle_cluster_tolerance_);
	//handle_cluster_.setSpatialLocator(0);
	handle_cluster_.setMinClusterSize(handle_cluster_min_size_);
	//handle_cluster_.setMaxClusterSize(handle_cluster_max_size_);
	handle_cluster_.setSearchMethod(clusters_tree_);

	seg_line_.setModelType(pcl::SACMODEL_LINE);
	seg_line_.setMethodType(pcl::SAC_RANSAC);
	seg_line_.setDistanceThreshold(0.05);
	seg_line_.setOptimizeCoefficients(true);
	seg_line_.setMaxIterations(max_iter_);
	seg_line_.setProbability(seg_prob_);

	min_table_inliers_ = 100;
	voxel_size_ = 0.01;

	k_ = 30;
	//normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> >();
	KdTreePtr normals_tree_(new KdTree);
	n3d_.setKSearch(k_);
	n3d_.setSearchMethod(normals_tree_);

	z_min_limit_ = 0.1;
	z_max_limit_ = 3.0;
	y_min_limit_ = -0.5;
	y_max_limit_ = 0.5;
	x_min_limit_ = 0.0;
	x_max_limit_ = 1.0;

	PointCloudPtr cloud_x_ptr(new PointCloud());
	PointCloudPtr cloud_y_ptr(new PointCloud());
	PointCloudPtr cloud_z_ptr(new PointCloud());

	//Downsample
	vgrid_.setInputCloud(cloudInput);
	vgrid_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
	//Filter x
	vgrid_.setFilterFieldName("x");
	//vgrid_.setFilterLimits(x_min_limit_, x_max_limit_);
	vgrid_.filter(*cloud_x_ptr);
	//Filter y
	vgrid_.setInputCloud(cloud_x_ptr);
	vgrid_.setFilterFieldName("y");
	//vgrid_.setFilterLimits(y_min_limit_, y_max_limit_);
	vgrid_.filter(*cloud_y_ptr);
	//Filter z
	vgrid_.setInputCloud(cloud_y_ptr);
	vgrid_.setFilterFieldName("z");
	//vgrid_.setFilterLimits(z_min_limit_, z_max_limit_);
	vgrid_.filter(*cloud_z_ptr);

	//Estimate Point Normals
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
	n3d_.setInputCloud(cloud_z_ptr);
	n3d_.compute(*cloud_normals);

	//Segment the biggest furniture_face plane
	pcl::ModelCoefficients::Ptr table_coeff(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
	seg_.setInputCloud(cloud_z_ptr);
	seg_.setInputNormals(cloud_normals);
	seg_.segment(*table_inliers, *table_coeff);
	ROS_INFO("Table model: [%f, %f, %f, %f] with %d inliers.",
			table_coeff->values[0], table_coeff->values[1],
			table_coeff->values[2], table_coeff->values[3], (int)table_inliers->indices.size ());

	if ((int) table_inliers->indices.size() <= min_table_inliers_) {
		ROS_ERROR("table has to few inliers");
		return;
	}

	//Extract the biggest cluster correponding to above inliers
	std::vector<pcl::PointIndices> clusters;
	cluster_.setInputCloud(cloud_z_ptr);
	cluster_.setIndices(table_inliers);
	cluster_.extract(clusters);

	PointCloudPtr biggest_face(new PointCloud());
	if (int(clusters.size()) >= nr_cluster_) {
		for (int i = 0; i < nr_cluster_; i++) {
			pcl::copyPointCloud(*cloud_z_ptr, clusters[i], *biggest_face);
		}
	} else {
		ROS_ERROR(
				"Only %ld clusters found with size > %d points", clusters.size(), object_cluster_min_size_);
		return;
	}
	ROS_INFO(
			"Found biggest face with %ld points", biggest_face->points.size());

	//Project Points into the Perfect plane
	PointCloudPtr cloud_projected(new PointCloud());
	proj_.setInputCloud(biggest_face);
	proj_.setModelCoefficients(table_coeff);
	proj_.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
	proj_.filter(*cloud_projected);

	PointCloudPtr cloud_hull(new PointCloud());
	// Create a Convex Hull representation of the projected inliers
	chull_.setInputCloud(cloud_projected);
	chull_.reconstruct(*cloud_hull);
	ROS_INFO(
			"Convex hull has: %d data points.", (int)cloud_hull->points.size ());
	if ((int) cloud_hull->points.size() == 0) {
		ROS_WARN(
				"Convex hull has: %d data points. Returning.", (int)cloud_hull->points.size ());
		return;
	}

	// Extract the handle clusters using a polygonal prism
	pcl::PointIndices::Ptr handles_indices(new pcl::PointIndices());
	prism_.setHeightLimits(cluster_min_height_, cluster_max_height_);
	//prism_.setInputCloud (cloud_z_ptr);
	prism_.setInputCloud(cloudInput);
	prism_.setInputPlanarHull(cloud_hull);
	prism_.segment(*handles_indices);
	ROS_INFO("Number of handle candidates: %d.", (int)handles_indices->indices.size ());

	//Cluster handles
	PointCloudPtr handles(new PointCloud());
	pcl::copyPointCloud(*cloudInput, *handles_indices, *handles);

	pcl::PCDWriter writer;
	writer.write("hull.pcd", *cloudInput, handles_indices->indices, true);
	writer.write("plane.pcd", *cloud_z_ptr, table_inliers->indices, true);

	handle_cluster_.setInputCloud(cloudInput);
	handle_cluster_.setIndices(handles_indices);
	handle_cluster_.extract(handle_clusters);

	ROS_INFO("Found handle clusters: %d.", (int)handle_clusters.size ());
	if ((int) handle_clusters.size() == 0)
		return;

	std::cout << "seg fault check 0" << "\n";
	PointCloudPtr handle_final(new PointCloud());
	pcl::ModelCoefficients::Ptr line_coeff(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices());

	//fit lines, project points into perfect lines
	std::cout << "seg fault check 1" << "\n";
	for (int i = 0; i < (int) handle_clusters.size(); i++) {
		std::cout << "seg fault check 2" << "\n";
		pcl::copyPointCloud(*handles, handle_clusters[i], *handle_final);
		std::cout << "seg fault check 3" << "\n";
		seg_line_.setInputCloud(handle_final);
		std::cout << "seg fault check 4" << "\n";
		seg_line_.segment(*line_inliers, *line_coeff);
		std::cout << "seg fault check 5" << "\n";
		ROS_INFO("line_inliers %ld", line_inliers->indices.size());
		std::cout << "seg fault check 6" << "\n";
	}

}

int main(int argc, char** argv) {
	if (argc != 3) {
		std::cerr << "please provide 2 point clouds as arguments)" << std::endl;
		exit(0);
	}
	PointCloudPtr cloudSource(new PointCloud);
	PointCloudPtr cloudTarget(new PointCloud);

	//Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read(argv[1], *cloudSource);
	reader.read(argv[2], *cloudTarget);

	std::vector<pcl::PointIndices> sourceHandleClusters;
	std::vector<pcl::PointIndices> targetHandleClusters;

	extractHandles(cloudSource, sourceHandleClusters);

	int i=1;
	pcl::PCDWriter writer;
	for(std::vector<pcl::PointIndices>::iterator iterator_ = sourceHandleClusters.begin(); iterator_ != sourceHandleClusters.end(); ++iterator_) {
		std::cout << i++ << "\n";
		writer.write("handle1.pcd", *cloudSource, iterator_->indices, true);
	}

}
