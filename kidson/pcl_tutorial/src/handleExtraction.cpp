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

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGBNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudNormal;
typedef pcl::search::KdTree<Point> KdTree;
typedef KdTree::Ptr KdTreePtr;
//typedef pcl::KdTree<Point>::Ptr KdTreePtr;
//typedef typename pcl::search::Search<Point>::Ptr KdTreePtr;

void normalEstimation(PointCloud::Ptr& pointCloudIn, PointCloudNormal::Ptr& pointCloudOut)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud (pointCloudIn);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03);
  ne.compute (*pointCloudOut);
  pcl::copyPointCloud (*pointCloudIn, *pointCloudOut);
}

void extractHandles(PointCloud::Ptr& cloudInput, PointCloudNormal::Ptr& pointCloudNormals, std::vector<int>& handles) {
	// PCL objects
	//pcl::PassThrough<Point> vgrid_;                   // Filtering + downsampling object
	pcl::VoxelGrid<Point> vgrid_; // Filtering + downsampling object
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

	seg_line_.setModelType(pcl::SACMODEL_LINE);
	seg_line_.setMethodType(pcl::SAC_RANSAC);
	seg_line_.setDistanceThreshold(0.05);
	seg_line_.setOptimizeCoefficients(true);
	seg_line_.setMaxIterations(max_iter_);
	seg_line_.setProbability(seg_prob_);

	// Segment planes
	pcl::OrganizedMultiPlaneSegmentation<Point, PointNormal, pcl::Label> mps;
	ROS_INFO("Segmenting planes");
	mps.setMinInliers (20000);
	mps.setMaximumCurvature(0.02);
	mps.setInputNormals (pointCloudNormals);
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
		filename.str("");
		filename << "plane" << plane << ".pcd";
		writer.write(filename.str(), *cloudInput, regionPoints[plane].indices, true);
		ROS_INFO("Plane model: [%f, %f, %f, %f] with %d inliers.",
				planes_coeff[plane].values[0], planes_coeff[plane].values[1],
				planes_coeff[plane].values[2], planes_coeff[plane].values[3], (int)regionPoints[plane].indices.size ());

		//Project Points into the Perfect plane
		PointCloud::Ptr cloud_projected(new PointCloud());
		pcl::PointIndicesPtr cloudPlaneIndicesPtr(new pcl::PointIndices(regionPoints[plane]));
		pcl::ModelCoefficientsPtr coeff(new pcl::ModelCoefficients(planes_coeff[plane]));
		proj_.setInputCloud(cloudInput);
		proj_.setIndices(cloudPlaneIndicesPtr);
		proj_.setModelCoefficients(coeff);
		proj_.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
		proj_.filter(*cloud_projected);

		PointCloud::Ptr cloud_hull(new PointCloud());
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

		/*######### handle clustering code
		handle_clusters.clear();
		handle_cluster_.setClusterTolerance(0.03);
		handle_cluster_.setMinClusterSize(200);
		handle_cluster_.setSearchMethod(clusters_tree_);
		handle_cluster_.setInputCloud(handles);
		handle_cluster_.setIndices(handlesIndicesPtr);
		handle_cluster_.extract(handle_clusters);
		for(size_t j = 0; j < handle_clusters.size(); j++)
		{
			filename.str("");
			filename << "handle" << (int)i << "-" << (int)j << ".pcd";
			writer.write(filename.str(), *handles, handle_clusters[j].indices, true);
		}*/

		pcl::StatisticalOutlierRemoval<Point> sor;
		PointCloud::Ptr cloud_filtered (new pcl::PointCloud<Point>);
		sor.setInputCloud (cloudInput);
		sor.setIndices(handlesIndicesPtr);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		sor.filter (*cloud_filtered);
		pcl::KdTreeFLANN<Point> kdtreeNN;
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

int main(int argc, char** argv) {
	if (argc != 3) {
		std::cerr << "please provide 2 point clouds as arguments)" << std::endl;
		exit(0);
	}
	PointCloud::Ptr cloudSource(new PointCloud);
	PointCloud::Ptr cloudTarget(new PointCloud);
	PointCloudNormal::Ptr cloudSourceNormal(new PointCloudNormal);
	PointCloudNormal::Ptr cloudTargetNormal(new PointCloudNormal);
	std::vector<int> indicesSource, indicesTarget;

	//Fill in the cloud data
	ROS_INFO("Reading files....");
	pcl::PCDReader reader;
	reader.read(argv[1], *cloudSource);
	reader.read(argv[2], *cloudTarget);

	ROS_INFO("Calculating normals....");
	normalEstimation(cloudSource, cloudSourceNormal);
	normalEstimation(cloudTarget, cloudTargetNormal);

	std::vector<int> sourceHandleClusters;
	std::vector<int> targetHandleClusters;

	ROS_INFO("Extracting handles....");
	extractHandles(cloudSource, cloudSourceNormal, sourceHandleClusters);
	extractHandles(cloudTarget, cloudTargetNormal, targetHandleClusters);

	 /*boost::shared_ptr< TransformationEstimationWDF<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal> >
	initialTransformWDF(new TransformationEstimationWDF<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal>());

	float alpha = 1.0;
	initialTransformWDF->setAlpha(alpha);
	initialTransformWDF->setCorrespondecesDFP(indicesSource, indicesTarget);

	// Instantiate ICP
	pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp_wdf;

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp_wdf.setMaxCorrespondenceDistance (0.05);
	// Set the maximum number of iterations (criterion 1)
	icp_wdf.setMaximumIterations (75);
	// Set the transformation epsilon (criterion 2)
	icp_wdf.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp_wdf.setEuclideanFitnessEpsilon (0); //1

	// Set TransformationEstimationWDF as ICP transform estimator
	icp_wdf.setTransformationEstimation (initialTransformWDF);

	icp_wdf.setInputCloud( concatinatedSourceCloud);
	icp_wdf.setInputTarget( concatinatedTargetCloud);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_transformed( new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	// As before, due to my initial bad naming, it is the "target" that is being transformed
	//									set initial transform
	icp_wdf.align ( *cloud_transformed, ransacInverse); //init_tr );
	std::cout << "[SIIMCloudMatch::runICPMatch] Has converged? = " << icp_wdf.hasConverged() << std::endl <<
				"	fitness score (SSD): " << icp_wdf.getFitnessScore (1000) << std::endl;
	icp_wdf.getFinalTransformation ();
*/
	pcl::PCDWriter writer;
	writer.write("handlesSource.pcd", *cloudSource, sourceHandleClusters, true);
	writer.write("handlesTarget.pcd", *cloudTarget, targetHandleClusters, true);
}
