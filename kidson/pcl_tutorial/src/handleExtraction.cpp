#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;

int
main (int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "please provide 2 point clouds as arguments)" << std::endl;
    exit(0);
  }
  PointCloudPtr cloudSource (new PointCloud);
  PointCloudPtr cloudTarget (new PointCloud);

  //Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read (argv[1], *cloudSource);
  reader.read (argv[2], *cloudTarget);


}


void extractHandles(PointCloudPtr& cloudInput, std::vector<int>& indicesOutput)
{
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
	KdTreePtr clusters_tree_, normals_tree_;

	PointCloudPtr cloud_x_ptr(new PointCloud());
	PointCloudPtr cloud_y_ptr(new PointCloud());
	PointCloudPtr cloud_z_ptr(new PointCloud());

	//Downsample
	vgrid_.setInputCloud(cloudInput);
	vgrid_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
	//Filter x
	vgrid_.setFilterFieldName("x");
	vgrid_.setFilterLimits(x_min_limit_, x_max_limit_);
	vgrid_.filter(*cloud_x_ptr);
	//Filter y
	vgrid_.setInputCloud(cloud_x_ptr);
	vgrid_.setFilterFieldName("y");
	vgrid_.setFilterLimits(y_min_limit_, y_max_limit_);
	vgrid_.filter(*cloud_y_ptr);
	//Filter z
	vgrid_.setInputCloud(cloud_y_ptr);
	vgrid_.setFilterFieldName("z");
	vgrid_.setFilterLimits(z_min_limit_, z_max_limit_);
	vgrid_.filter(*cloud_z_ptr);

	//Estimate Point Normals
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
			new pcl::PointCloud<pcl::Normal>());
	n3d_.setInputCloud(cloud_z_ptr);
	n3d_.compute(*cloud_normals);

	//Segment the biggest furniture_face plane
	pcl::ModelCoefficients::Ptr table_coeff(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
	seg_.setInputCloud(cloud_z_ptr);
	seg_.setInputNormals(cloud_normals);
	seg_.segment(*table_inliers, *table_coeff);
	ROS_INFO(
			"[%s] Table model: [%f, %f, %f, %f] with %d inliers.", getName ().c_str (), table_coeff->values[0], table_coeff->values[1], table_coeff->values[2], table_coeff->values[3], (int)table_inliers->indices.size ());

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
	//For Debug
	//cloud_pub_.publish(*biggest_face);
	//return;

	//Project Points into the Perfect plane
	PointCloudPtr cloud_projected(new PointCloud());
	proj_.setInputCloud(biggest_face);
	proj_.setModelCoefficients(table_coeff);
	proj_.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
	proj_.filter(*cloud_projected);
	//For Debug
	//cloud_pub_.publish(*cloud_projected);
	//return;

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
	//For Debug
	cloud_pub_.publish(*cloud_hull);
	//return;

	// Extract the handle clusters using a polygonal prism
	pcl::PointIndices::Ptr handles_indices(new pcl::PointIndices());
	prism_.setHeightLimits(cluster_min_height_, cluster_max_height_);
	//prism_.setInputCloud (cloud_z_ptr);
	prism_.setInputCloud(cloudInput);
	prism_.setInputPlanarHull(cloud_hull);
	prism_.segment(*handles_indices);
	ROS_INFO(
			"[%s] Number of handle candidates: %d.", getName ().c_str (), (int)handles_indices->indices.size ());

	//Cluster handles
	PointCloudPtr handles(new PointCloud());
	pcl::copyPointCloud(*cloudInput, *handles_indices, *handles);
	//For Debug
	//cloud_pub_.publish(*handles);
	//return;

	 std::vector<pcl::PointIndices> handle_clusters;
	        //      handle_cluster_.setInputCloud(handles);
	        handle_cluster_.setInputCloud(*cloud_raw_ptr);
	        handle_cluster_.setIndices(*handles_indices);
	        handle_cluster_.extract(handle_clusters);


	std::vector<pcl::PointIndices> handle_clusters;
	handle_cluster_.setInputCloud(handles);
	handle_cluster_.extract(handle_clusters);
	ROS_INFO(
			"[%s] Found handle clusters: %d.", getName ().c_str (), (int)handle_clusters.size ());
	if ((int) handle_clusters.size() == 0) {
	  result_.number_of_handles_detected = 0;
	  result_.handles.resize(0);
	  as_.setSucceeded(result_);
		return;
	}

	PointCloudPtr handle_final(new PointCloud());
	pcl::ModelCoefficients::Ptr line_coeff(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices());

	// determine the number of handles that will be returned
	int handle_clusters_size;
	if ((int)handle_clusters.size() < goal->number_of_handles) {
		handle_clusters_size = handle_clusters.size();
	} else {
		handle_clusters_size = goal->number_of_handles;
	}

	// init the result message accordingly
	result_.number_of_handles_detected = handle_clusters_size;
	result_.handles.resize(0);

	//fit lines, project points into perfect lines
	for (int i = 0; i < handle_clusters_size; i++) {
		pcl::copyPointCloud(*handles, handle_clusters[i], *handle_final);
		seg_line_.setInputCloud(handle_final);
		seg_line_.segment(*line_inliers, *line_coeff);
		ROS_INFO("line_inliers %ld", line_inliers->indices.size());
	}


}
