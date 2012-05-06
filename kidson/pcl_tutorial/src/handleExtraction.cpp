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
#include "pcl/segmentation/organized_multi_plane_segmentation.h"
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

void normalEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudIn, pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (pointCloudIn);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03);
  ne.compute (*normals);
}

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
	pcl::PCDWriter writer;

	double sac_distance_, normal_distance_weight_;
	double eps_angle_, seg_prob_;
	int max_iter_;

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

	cluster_.setClusterTolerance(0.03);
	cluster_.setMinClusterSize(200);
	KdTreePtr clusters_tree_(new KdTree);
	clusters_tree_->setEpsilon(1);
	cluster_.setSearchMethod(clusters_tree_);

	handle_cluster_.setClusterTolerance(0.03);
	handle_cluster_.setMinClusterSize(200);
	handle_cluster_.setSearchMethod(clusters_tree_);

	seg_line_.setModelType(pcl::SACMODEL_LINE);
	seg_line_.setMethodType(pcl::SAC_RANSAC);
	seg_line_.setDistanceThreshold(0.05);
	seg_line_.setOptimizeCoefficients(true);
	seg_line_.setMaxIterations(max_iter_);
	seg_line_.setProbability(seg_prob_);

	KdTreePtr normals_tree_(new KdTree);
	n3d_.setKSearch(100);
	n3d_.setSearchMethod(normals_tree_);

	//Estimate Point Normals
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normalss(new pcl::PointCloud<pcl::Normal>());
	normalEstimation(cloudInput, cloud_normalss);

	// Segment planes
	pcl::OrganizedMultiPlaneSegmentation<Point, pcl::Normal, pcl::Label> mps;
	mps.setMinInliers (20000);
	mps.setMaximumCurvature(0.02);
	mps.setInputNormals (cloud_normalss);
	mps.setInputCloud (cloudInput);
	std::vector<pcl::PlanarRegion<Point> > regions;
	std::vector<pcl::PointIndices> regionPoints;
	std::vector< pcl::ModelCoefficients > planes_coeff;
	//mps.segmentAndRefine (regions);
	mps.segment(planes_coeff, regionPoints);
	//ROS_INFO_STREAM("Number of regions:" << regions.size());
	ROS_INFO_STREAM("Number of regions:" << regionPoints.size());

	if ((int) regionPoints.size() < 1) {
		ROS_ERROR("no planes found");
		return;
	}

  	std::stringstream filename;
	for (size_t i = 0; i < regionPoints.size (); i++)
	{
		filename.str("");
		filename << "plane" << i << ".pcd";
		writer.write(filename.str(), *cloudInput, regionPoints[i].indices, true);
		ROS_INFO("Plane model: [%f, %f, %f, %f] with %d inliers.",
				planes_coeff[i].values[0], planes_coeff[i].values[1],
				planes_coeff[i].values[2], planes_coeff[i].values[3], (int)regionPoints[i].indices.size ());

		//Project Points into the Perfect plane
		PointCloudPtr cloud_projected(new PointCloud());
		pcl::PointIndicesPtr cloudPlaneIndicesPtr(new pcl::PointIndices(regionPoints[i]));
		pcl::ModelCoefficientsPtr coeff(new pcl::ModelCoefficients(planes_coeff[i]));
		proj_.setInputCloud(cloudInput);
		proj_.setIndices(cloudPlaneIndicesPtr);
		proj_.setModelCoefficients(coeff);
		proj_.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
		proj_.filter(*cloud_projected);

		PointCloudPtr cloud_hull(new PointCloud());
		// Create a Convex Hull representation of the projected inliers
		chull_.setInputCloud(cloud_projected);
		chull_.reconstruct(*cloud_hull);
		ROS_INFO("Convex hull has: %d data points.", (int)cloud_hull->points.size ());
		if ((int) cloud_hull->points.size() == 0)
		{
			ROS_WARN("Convex hull has: %d data points. Returning.", (int)cloud_hull->points.size ());
			return;
		}

		// Extract the handle clusters using a polygonal prism
		pcl::PointIndices::Ptr handlesIndicesPtr(new pcl::PointIndices());
		prism_.setHeightLimits(0.03, 0.1);
		prism_.setInputCloud(cloudInput);
		prism_.setInputPlanarHull(cloud_hull);
		prism_.segment(*handlesIndicesPtr);

		//Cluster handles
		PointCloudPtr handles(new PointCloud());
		pcl::copyPointCloud(*cloudInput, *handlesIndicesPtr, *handles);

		filename.str("");
		filename << "xhull" << i << ".pcd";			  // prefix x so it appears at the bottom of the list with ls
		writer.write(filename.str(), *cloudInput, handlesIndicesPtr->indices, true);

		ROS_INFO("Number of handle candidates: %d.", (int)handlesIndicesPtr->indices.size ());
		std::vector<pcl::PointIndices> handle_clusters_local;
		if((int)handlesIndicesPtr->indices.size () > 500)
		{
			//handle_cluster_.setInputCloud(cloudInput);
			//handle_cluster_.setIndices(handles_indices);
			handle_clusters.clear();
			handle_cluster_.setInputCloud(handles);
			//handle_cluster_.setIndices(handlesIndicesPtr);
			handle_cluster_.extract(handle_clusters_local);
		/*	for(size_t i = 0; i < handle_clusters_local.size(); i++)
			{
				pcl::PointIndices handleIndicesTemp;
				for(size_t j = 0; j < handle_clusters_local[i].indices.size(); i++)
				{
					for(size_t k = 0; k < cloudInput->points.size(); i++)
					{
						if(handles->points[j].x == cloudInput->points[k].x
								&& handles->points[j].y == cloudInput->points[k].y
								&& handles->points[j].z == cloudInput->points[k].z)
						{
							handleIndicesTemp.addIndices(k);
							k = cloudInput->points.size();
						}
					}
				}
				handle_clusters.push_back(handleIndicesTemp);
			}*/
		}
		else
			ROS_INFO("Not enough points to look for handles");

		ROS_INFO("Found handle clusters: %d.", (int)handle_clusters.size ());
		//if ((int) handle_clusters.size() == 0)
		//	return;
	}

	/*
	std::cout << "seg fault check 0" << "\n";
	PointCloudPtr handle_final(new PointCloud());
	pcl::ModelCoefficients::Ptr line_coeff(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices());

	//fit lines, project points into perfect lines
	std::cout << "seg fault check 1" << "\n";
	//for (int i = 0; i < (int) handle_clusters.size(); i++) {
	for(std::vector<pcl::PointIndices>::iterator iterator_ = handle_clusters.begin(); iterator_ != handle_clusters.end(); ++iterator_) {
		pcl::copyPointCloud(*cloudInput, *iterator_, *handle_final);
		seg_line_.setInputCloud(handle_final);
		seg_line_.segment(*line_inliers, *line_coeff);
		ROS_INFO("line_inliers %ld", line_inliers->indices.size());
	}*/
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
