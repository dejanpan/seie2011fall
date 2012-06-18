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
#include <pcl/registration/transformation_estimation_joint_optimize.h>
#include <pcl/registration/icp_joint_optimize.h>

#include "tf/tf.h"

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGBNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudNormal;
typedef pcl::search::KdTree<Point> KdTree;
typedef KdTree::Ptr KdTreePtr;
//typedef pcl::KdTree<Point>::Ptr KdTreePtr;
//typedef typename pcl::search::Search<Point>::Ptr KdTreePtr;

void normalEstimation(PointCloud::Ptr& pointCloudIn, PointCloudNormal::Ptr& pointCloudOut, float radiusSeachSize)
{
  // Create the normal estimation class, and pass the input dataset to it
  ROS_INFO("Calculating normals....");
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud (pointCloudIn);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (radiusSeachSize);
  ne.compute (*pointCloudOut);
  pcl::copyPointCloud (*pointCloudIn, *pointCloudOut);
}
// input: cloudInput
// input: pointCloudNormals
// output: cloudOutput
// output: pointCloudNormalsFiltered
void extractHandles(PointCloud::Ptr& cloudInput, PointCloud::Ptr& cloudOutput, PointCloudNormal::Ptr& pointCloudNormals, std::vector<int>& handles) {
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

	//filter cloud
	double leafSize = 0.001;
	pcl::VoxelGrid<Point> sor;
	sor.setInputCloud (cloudInput);
	sor.setLeafSize (leafSize, leafSize, leafSize);
	sor.filter (*cloudOutput);
	//sor.setInputCloud (pointCloudNormals);
	//sor.filter (*pointCloudNormalsFiltered);
	//std::vector<int> tempIndices;
	//pcl::removeNaNFromPointCloud(*cloudInput, *cloudOutput, tempIndices);
	//pcl::removeNaNFromPointCloud(*pointCloudNormals, *pointCloudNormalsFiltered, tempIndices);

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
		prism_.setHeightLimits(0.05, 0.1);
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
		PointCloudNormal::Ptr cloud_filtered_hack (new PointCloudNormal);
		pcl::copyPointCloud(*cloud_filtered, *cloud_filtered_hack);

		// Our handle is in cloud_filtered.  We want indices of a cloud (also filtered for NaNs)
		pcl::KdTreeFLANN<PointNormal> kdtreeNN;
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		kdtreeNN.setInputCloud(pointCloudNormals);
		for(size_t j = 0; j < cloud_filtered_hack->points.size(); j++)
		{
			kdtreeNN.nearestKSearch(cloud_filtered_hack->points[j], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
			handles.push_back(pointIdxNKNSearch[0]);
		}
	}
}

void removeNaNs(const PointCloudNormal::Ptr& cloudInput, PointCloudNormal::Ptr& cloudOutput, std::vector<int>& removedPoints)
{
	removedPoints.clear();
	cloudOutput->header   = cloudInput->header;

	for(size_t i=0; i < cloudInput->points.size(); i++)
	{
		if(!pcl_isfinite(cloudInput->points[i].x) ||
				!pcl_isfinite(cloudInput->points[i].y) ||
				!pcl_isfinite(cloudInput->points[i].z))
		{
//			ROS_INFO_STREAM("NAN FOUND IN POINT COORDINATE. " << i);
			removedPoints.push_back((int)i);
		}
		else if(!pcl_isfinite(cloudInput->points[i].normal_x) ||
				!pcl_isfinite(cloudInput->points[i].normal_y) ||
				!pcl_isfinite(cloudInput->points[i].normal_z))
		{
			ROS_INFO_STREAM("NAN FOUND IN NORMAL. Specify larger K or search radius to calculate normals " << i);
			removedPoints.push_back(int(i));
		}

		else
			cloudOutput->points.push_back(cloudInput->points[i]);
	}
	cloudOutput->height = 1;
	cloudOutput->width  = static_cast<uint32_t>(cloudOutput->points.size());
	ROS_INFO_STREAM("Size of removed indices" << removedPoints.size());
}

void adjustIndicesFromRemovedPoints(std::vector<int>& indicesInput, std::vector<int>& indicesOutput, const std::vector<int>& removedPoints)
{
	// look for indices that were removed and remove them from the vector
	indicesOutput = indicesInput;
	for(size_t i=0; i < removedPoints.size(); i++)
	{
		std::vector<int>::iterator j = std::find(indicesOutput.begin(), indicesOutput.end(), removedPoints[i]);
		if(*j == removedPoints[i])	//check if search successful
		{
			indicesOutput.erase(j);
		}
	}
	for(size_t i=0; i < removedPoints.size(); i++)
	{
//		ROS_INFO_STREAM("removed points[" << i << "] = " << removedPoints[i] << "######################################################################");
		for(size_t j=0; j < indicesInput.size(); j++)
		{
			if(indicesInput[j] > removedPoints[i])
			{
				indicesOutput[j]--;
//				ROS_INFO_STREAM("indice " << j << " was decremented to " << indicesInput[j]);
			}
		}
	}
}


int main(int argc, char** argv) {
	if (argc != 3) {
		std::cerr << "please provide 2 point clouds as arguments: <source>.pcd  <target>.pcd)" << std::endl;
		exit(0);
	}
	PointCloud::Ptr cloudSource(new PointCloud);
	PointCloud::Ptr cloudOut(new PointCloud);
	PointCloud::Ptr cloudTarget(new PointCloud);
	PointCloud::Ptr cloudSourceFiltered(new PointCloud);
	PointCloud::Ptr cloudTargetFiltered(new PointCloud);
	PointCloudNormal::Ptr cloudSourceNormal(new PointCloudNormal);
	PointCloudNormal::Ptr cloudTargetNormal(new PointCloudNormal);
	PointCloudNormal::Ptr cloudSourceNormalFiltered(new PointCloudNormal);
	PointCloudNormal::Ptr cloudTargetNormalFiltered(new PointCloudNormal);
	PointCloudNormal::Ptr cloudSourceNormalNoNaNs(new PointCloudNormal);
	PointCloudNormal::Ptr cloudTargetNormalNoNaNs(new PointCloudNormal);
	pcl::PCDWriter writer;
	std::vector<int> indicesSource, indicesTarget;

	//Fill in the cloud data
	ROS_INFO("Reading files....");
	pcl::PCDReader reader;
	reader.read(argv[1], *cloudSource);
	reader.read(argv[2], *cloudTarget);
	//pcl::copyPointCloud (*cloudSourceNormal, *cloudSource);
	//pcl::copyPointCloud (*cloudTargetNormal, *cloudTarget);

	normalEstimation(cloudSource, cloudSourceNormal, 0.03);
	normalEstimation(cloudTarget, cloudTargetNormal, 0.03);

	std::vector<int> sourceHandleClusters;
	std::vector<int> targetHandleClusters;
	std::vector<int> sourceHandleClustersNoNaNs;
	std::vector<int> targetHandleClustersNoNaNs;

	ROS_INFO("Extracting handles....");
	extractHandles(cloudSource, cloudSourceFiltered, cloudSourceNormal, sourceHandleClusters);
	extractHandles(cloudTarget, cloudTargetFiltered, cloudTargetNormal, targetHandleClusters);

	std::vector<int> removedPoints;
	removeNaNs(cloudSourceNormal, cloudSourceNormalNoNaNs, removedPoints);
	adjustIndicesFromRemovedPoints(sourceHandleClusters, sourceHandleClustersNoNaNs, removedPoints);
	removeNaNs(cloudTargetNormal, cloudTargetNormalNoNaNs, removedPoints);
	adjustIndicesFromRemovedPoints(targetHandleClusters, targetHandleClustersNoNaNs, removedPoints);

//	normalEstimation(cloudSourceFiltered, cloudSourceNormalFiltered, 0.035);//0.05
//	normalEstimation(cloudTargetFiltered, cloudTargetNormalFiltered, 0.035);//0.05

//	for(size_t i=0; i < cloudSourceNormalFiltered->points.size(); i++)
//	{
//		if((cloudSourceNormalFiltered->points[i].normal_x != cloudSourceNormalFiltered->points[i].normal_x) ||
//				(cloudSourceNormalFiltered->points[i].normal_y != cloudSourceNormalFiltered->points[i].normal_y) ||
//				(cloudSourceNormalFiltered->points[i].normal_z != cloudSourceNormalFiltered->points[i].normal_z))
//		{
//			std::cerr << "NAN IN SOURCE NORMAL. Specify larger K or search radius to calculate normals " << i << "\n";
//			cloudSourceNormalFiltered->points.erase(cloudSourceNormalFiltered->points.begin() + i);
//			for(size_t j=0; j < sourceHandleClusters.size(); j++)
//			{
//				if(sourceHandleClusters[j] > (int)i)
//				{
//					sourceHandleClusters[j]--;
//					std::cerr << "source handle indice " << j << " was decremented to " << sourceHandleClusters[j] << "\n";
//				}
//			}
//			i--;
//			//exit(0);
//		}
//	}
//	for(size_t i=0; i < cloudTargetNormalFiltered->points.size(); i++)
//	{
//		if((cloudTargetNormalFiltered->points[i].normal_x != cloudTargetNormalFiltered->points[i].normal_x) ||
//				(cloudTargetNormalFiltered->points[i].normal_y != cloudTargetNormalFiltered->points[i].normal_y) ||
//				(cloudTargetNormalFiltered->points[i].normal_z != cloudTargetNormalFiltered->points[i].normal_z))
//		{
//			std::cerr << "NAN IN TARGET NORMAL. Specify larger K or search radius to calculate normals " << i << "\n";
//			cloudTargetNormalFiltered->points.erase(cloudTargetNormalFiltered->points.begin() + i);
//			for(size_t j=0; j < targetHandleClusters.size(); j++)
//			{
//				if(targetHandleClusters[j] > (int)i)
//				{
//					targetHandleClusters[j]--;
//					std::cerr << "target handle indice " << j << " was decremented to " << targetHandleClusters[j] << "\n";
//				}
//			}
//			i--;
//			//exit(0);
//		}
//	}

	writer.write("handlesSource.pcd", *cloudSourceNormalNoNaNs, sourceHandleClustersNoNaNs, true);
	writer.write("handlesTarget.pcd", *cloudTargetNormalNoNaNs, targetHandleClustersNoNaNs, true);

	ROS_INFO("Initialize transformation estimation object....");
	boost::shared_ptr< TransformationEstimationJointOptimize<PointNormal, PointNormal > >
		transformationEstimation_(new TransformationEstimationJointOptimize<PointNormal, PointNormal>());

	float denseCloudWeight = 1.0;
	float visualFeatureWeight = 0.0;
	float handleFeatureWeight = 0.25;
	transformationEstimation_->setWeights(denseCloudWeight, visualFeatureWeight, handleFeatureWeight);
	transformationEstimation_->setCorrespondecesDFP(indicesSource, indicesTarget);

	 Eigen::Matrix4f guess;
	//  guess << 0.999203,   0.0337772,  -0.0213298,   0.0110106,
	//		  -0.0349403,     0.99778,  -0.0567293, -0.00746282,
	//		  0.0193665,   0.0574294,    0.998162,   0.0141431,
	//			  0,           0,           0,           1;
//	  guess << 1,   0.2,  0.3,  -0.3,
//			   -0.2,	1,	-0.2,	0.9,
//			   -0.3,	0.2,	1,	0.4,
//			   0,	0,	0,	1;

//	  guess << 1,   0,  0,  0.0,
//			   0,	1,	0,	0.5,
//			   0,	0,	1,	0.33,
//			   0,	0,	0,	1;

	 guess <<   0.993523,  0.0152363,  -0.112636,   0.138385,
	 	-0.0264756,   0.994739, -0.0989777,   0.615225,
	 	  0.110535,   0.101318,   0.988696,   0.347863,
	 	         0,          0,          0,          1;
	// custom icp
	ROS_INFO("Initialize icp object....");
	pcl::IterativeClosestPointJointOptimize<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icpJointOptimize; //JointOptimize
	icpJointOptimize.setMaximumIterations (40);
	icpJointOptimize.setTransformationEpsilon (0);
	icpJointOptimize.setMaxCorrespondenceDistance(0.1);
	icpJointOptimize.setRANSACOutlierRejectionThreshold(0.03);
	icpJointOptimize.setEuclideanFitnessEpsilon (0);
	icpJointOptimize.setTransformationEstimation (transformationEstimation_);
	icpJointOptimize.setHandleSourceIndices(sourceHandleClustersNoNaNs);
	icpJointOptimize.setHandleTargetIndices(targetHandleClustersNoNaNs);
	icpJointOptimize.setInputCloud(cloudSourceNormalNoNaNs);
	icpJointOptimize.setInputTarget(cloudTargetNormalNoNaNs);

	ROS_INFO("Running ICP....");
	PointCloudNormal::Ptr cloud_transformed( new PointCloudNormal);
	icpJointOptimize.align ( *cloud_transformed, guess); //init_tr );
	std::cout << "[SIIMCloudMatch::runICPMatch] Has converged? = " << icpJointOptimize.hasConverged() << std::endl <<
				"	fitness score (SSD): " << icpJointOptimize.getFitnessScore (1000) << std::endl
				<<	icpJointOptimize.getFinalTransformation () << "\n";
	transformPointCloud (*cloudSource, *cloudOut,  icpJointOptimize.getFinalTransformation());

	ROS_INFO("Writing output....");
	writer.write("converged.pcd", *cloudOut, true);
}

/*
 * Run ICP with point to plane error metric
has converged:1 score: 0.00104609
     0.98492     0.171598   -0.0220549    0.0408765
   -0.172011     0.984923   -0.0184277   -0.0478952
   0.0185603    0.0219435     0.999587 -0.000478007
           0            0            0            1
 *


0.992429      0.11531   -0.0422939    0.0398834
-0.117302      0.99193   -0.0481167   -0.0659994
0.0364043    0.0527136     0.997946 -0.000340177
       0            0            0            1

as converged:1 score: 0.00134764
    0.992429      0.11531   -0.0422939    0.0398834
   -0.117302      0.99193   -0.0481167   -0.0659994
   0.0364043    0.0527136     0.997946 -0.000340177
           0            0            0            1


       */
