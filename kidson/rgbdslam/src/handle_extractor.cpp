/*
 * HandleExtractor.cpp
 *
 *  Created on: 31.05.2012
 *      Author: ross
 */

#include "handle_extractor.h"
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
#include <pcl/filters/statistical_outlier_removal.h>

#include "tf/tf.h"

HandleExtractor::HandleExtractor() {
	// TODO Auto-generated constructor stub

}

HandleExtractor::~HandleExtractor() {
	// TODO Auto-generated destructor stub
}

void HandleExtractor::extractHandles(PointCloudNormal::Ptr& cloudInput, std::vector<int>& handles) {
	// PCL objects
	//pcl::PassThrough<Point> vgrid_;                   // Filtering + downsampling object
	pcl::VoxelGrid<Point> vgrid_; // Filtering + downsampling object
	pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_; // Planar segmentation object
	pcl::SACSegmentation<Point> seg_line_; // Planar segmentation object
	pcl::ProjectInliers<PointNormal> proj_; // Inlier projection object
	pcl::ExtractIndices<Point> extract_; // Extract (too) big tables
	pcl::ConvexHull<PointNormal> chull_;
	pcl::ExtractPolygonalPrismData<PointNormal> prism_;
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

	seg_line_.setModelType(pcl::SACMODEL_LINE);
	seg_line_.setMethodType(pcl::SAC_RANSAC);
	seg_line_.setDistanceThreshold(0.05);
	seg_line_.setOptimizeCoefficients(true);
	seg_line_.setMaxIterations(max_iter_);
	seg_line_.setProbability(seg_prob_);

	// Segment planes
	pcl::OrganizedMultiPlaneSegmentation<PointNormal, PointNormal, pcl::Label> mps;
	ROS_INFO("Segmenting planes");
	mps.setMinInliers (20000);
	mps.setMaximumCurvature(0.02);
	mps.setInputNormals (cloudInput);
	mps.setInputCloud (cloudInput);
	std::vector<pcl::PlanarRegion<Point> > regions;
	std::vector<pcl::PointIndices> regionPoints;
	std::vector< pcl::ModelCoefficients > planes_coeff;
	mps.segment(planes_coeff, regionPoints);
	ROS_INFO_STREAM("Number of regions:" << regionPoints.size());

	if ((int) regionPoints.size() < 1) {
		ROS_ERROR("no planes found");
		return;
	}

  	std::stringstream filename;
	for (size_t plane = 0; plane < regionPoints.size (); plane++)
	{
		//filename.str("");
		//filename << "plane" << plane << ".pcd";
		//writer.write(filename.str(), *cloudInput, regionPoints[plane].indices, true);
		ROS_INFO("Plane model: [%f, %f, %f, %f] with %d inliers.",
				planes_coeff[plane].values[0], planes_coeff[plane].values[1],
				planes_coeff[plane].values[2], planes_coeff[plane].values[3], (int)regionPoints[plane].indices.size ());

		//Project Points into the Perfect plane
		PointCloudNormal::Ptr cloud_projected(new PointCloudNormal());
		pcl::PointIndicesPtr cloudPlaneIndicesPtr(new pcl::PointIndices(regionPoints[plane]));
		pcl::ModelCoefficientsPtr coeff(new pcl::ModelCoefficients(planes_coeff[plane]));
		proj_.setInputCloud(cloudInput);
		proj_.setIndices(cloudPlaneIndicesPtr);
		proj_.setModelCoefficients(coeff);
		proj_.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
		proj_.filter(*cloud_projected);

		PointCloudNormal::Ptr cloud_hull(new PointCloudNormal());
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

		ROS_INFO("Number of handle candidates: %d.", (int)handlesIndicesPtr->indices.size ());
		if((int)handlesIndicesPtr->indices.size () < 1100)
			continue;

		pcl::StatisticalOutlierRemoval<PointNormal> sor;
		PointCloudNormal::Ptr cloud_filtered (new pcl::PointCloud<PointNormal>);
		sor.setInputCloud (cloudInput);
		sor.setIndices(handlesIndicesPtr);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		sor.filter (*cloud_filtered);
		pcl::KdTreeFLANN<PointNormal> kdtreeNN;
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		kdtreeNN.setInputCloud(cloudInput);
		for(size_t j = 0; j < cloud_filtered->points.size(); j++)
		{
			kdtreeNN.nearestKSearch(cloud_filtered->points[j], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
			handles.push_back(pointIdxNKNSearch[0]);
		}
	}
}
