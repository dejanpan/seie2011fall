#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_features.h>
#include <pcl/features/normal_3d.h>
#include "pcl_ros/transforms.h"
#include <pcl/filters/voxel_grid.h>

//rosbag stuff:
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "pcl_tutorial/featureMatch.h"
#include "pcl_tutorial/match.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;

void normalEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudIn, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloudOut)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud (pointCloudIn);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.01);
  ne.compute (*pointCloudOut);
  pcl::copyPointCloud (*pointCloudIn, *pointCloudOut);
}

/** @brief Helper function to convert Eigen transformation to tf */
Eigen::Matrix4f EigenfromTf(tf::Transform trans)
{
	Eigen::Matrix4f eignMat;
	eignMat(0,3) = trans.getOrigin().getX();
	eignMat(1,3) = trans.getOrigin().getY();
	eignMat(2,3) = trans.getOrigin().getZ();
	for (int i=0;i<3;i++)
	{
		eignMat(i,0) = trans.getBasis().getRow(i).getX();
		eignMat(i,1) = trans.getBasis().getRow(i).getY();
		eignMat(i,2) = trans.getBasis().getRow(i).getZ();
	}
	eignMat(3,3) = 1;
	//ROS_INFO("trans: %f, %f, %f %f | %f, %f, %f %f | %f, %f, %f %f", eignMat(0,0), eignMat(0,1), eignMat(0,2), eignMat(0,3), eignMat(1,0), eignMat(1,1), eignMat(1,2), eignMat(1,3), eignMat(2,0), eignMat(2,1), eignMat(2,2), eignMat(2,3));
    return eignMat;
}

void voxFilterPointCloud(sensor_msgs::PointCloud2::Ptr cloudIn, sensor_msgs::PointCloud2::Ptr cloudOut)
{
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (cloudIn);
	sor.setLeafSize (0.001f, 0.001f, 0.001f);
	sor.filter (*cloudOut);
}

void getTestDataFromBag(PointCloudPtr cloud_source, PointCloudPtr cloud_target,
		PointCloudPtr featureCloudSource, std::vector<int> &indicesSource,
		PointCloudPtr featureCloudTarget, std::vector<int> &indicesTarget,
		Eigen::Matrix4f &initialTransform, int rosMessageNumber)
{
	sensor_msgs::PointCloud2::Ptr cloud_source_filtered (new sensor_msgs::PointCloud2 ());
	sensor_msgs::PointCloud2::Ptr cloud_target_filtered (new sensor_msgs::PointCloud2 ());



	rosbag::Bag bag;
	bag.open("test.bag", rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(std::string("/feature_match_out_topic"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	int i = 1;
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		if( i == rosMessageNumber)
		{
			pcl_tutorial::featureMatch::ConstPtr fm = m.instantiate<pcl_tutorial::featureMatch>();

			ROS_INFO("Converting point cloud message to local pcl clouds");

			sensor_msgs::PointCloud2::Ptr cloud_source_temp_Ptr (new sensor_msgs::PointCloud2 (fm->sourcePointcloud));
			sensor_msgs::PointCloud2::Ptr cloud_target_temp_Ptr (new sensor_msgs::PointCloud2 (fm->targetPointcloud));

			voxFilterPointCloud(cloud_source_temp_Ptr, cloud_source_filtered);
			voxFilterPointCloud(cloud_target_temp_Ptr, cloud_target_filtered);
			ROS_INFO("Converting dense clouds");
			pcl::fromROSMsg(*cloud_source_filtered, *cloud_source);
			pcl::fromROSMsg(*cloud_target_filtered, *cloud_target);
			ROS_INFO("Converting sparse clouds");
			pcl::fromROSMsg(fm->sourceFeatureLocations, *featureCloudSource);
			pcl::fromROSMsg(fm->targetFeatureLocations, *featureCloudTarget);

			ROS_INFO("Converting geometry message to eigen4f");
		    tf::Transform trans;
		    tf::transformMsgToTF(fm->featureTransform,trans);
		    initialTransform = EigenfromTf(trans);
		    ROS_INFO_STREAM("transform from ransac: " << "\n"  << initialTransform << "\n");

		    ROS_INFO("Extracting corresponding indices");
		    int j = 1;
		  	for(std::vector<pcl_tutorial::match>::const_iterator iterator_ = fm->matches.begin(); iterator_ != fm->matches.end(); ++iterator_)
		  	{
		  		indicesSource.push_back(iterator_->queryId);
		  		indicesTarget.push_back(iterator_->trainId);
		  		ROS_INFO_STREAM("source point " << j << ": "   << featureCloudSource->points[iterator_->queryId].x << ", " << featureCloudSource->points[iterator_->queryId].y << ", " << featureCloudSource->points[iterator_->queryId].z);
		  		ROS_INFO_STREAM("target point " << j++ << ": " << featureCloudTarget->points[iterator_->trainId].x << ", " << featureCloudTarget->points[iterator_->trainId].y << ", " << featureCloudTarget->points[iterator_->trainId].z);
		  	//	ROS_INFO("qidx: %d tidx: %d iidx: %d dist: %f", iterator_->queryId, iterator_->trainId, iterator_->imgId, iterator_->distance);
		  	}
		  	i++;

		  //  for(std::vector<int>::iterator iterator_ = indicesSource.begin(); iterator_ != indicesSource.end(); ++iterator_) {
		  //  	ROS_INFO("source indice: %d", *iterator_);
		  //  }
		}
		else
			i++;
	}
	bag.close();
}

int main (int argc, char** argv)
{
  PointCloudPtr cloud_source (new PointCloud);
  PointCloudPtr cloud_target (new PointCloud);
  PointCloudPtr featureCloudSourceTemp (new PointCloud);
  PointCloudPtr featureCloudTargetTemp (new PointCloud);
  PointCloudPtr cloud_converg_sparse (new PointCloud);
  PointCloudPtr cloud_converg_dense (new PointCloud);
  PointCloudNormalPtr cloud_source_normals (new PointCloudNormal);
  PointCloudNormalPtr cloud_target_normals (new PointCloudNormal);
  PointCloudNormalPtr featureCloudSource (new PointCloudNormal);
  PointCloudNormalPtr featureCloudTarget (new PointCloudNormal);
  Eigen::Matrix4f initialTransform;
  std::vector<int> indicesSource;
  std::vector<int> indicesTarget;

  //Fill in the cloud data
  //pcl::PCDReader reader;
  //ROS_INFO("Reading saved clouds with normals from file (faster)");
  //reader.read ("normals-source.pcd", *cloud_source_normals);
  //reader.read ("normals-target.pcd", *cloud_target_normals);
  //std::cout << "PointCloud source has: " << cloud_source->points.size () << " data points." << std::endl;
  //std::cout << "PointCloud target has: " << cloud_target->points.size () << " data points." << std::endl;


  ROS_INFO("Getting test data from a bag file");
  getTestDataFromBag(cloud_source, cloud_target, featureCloudSourceTemp, indicesSource, featureCloudTargetTemp, indicesTarget, initialTransform, 1);

  //calculate normals
  ROS_INFO("Calcualting normals");
 normalEstimation(cloud_source, cloud_source_normals);
 normalEstimation(cloud_target, cloud_target_normals);

  ROS_INFO("Converting feature point clouds");
  pcl::copyPointCloud (*featureCloudSourceTemp, *featureCloudSource);
  pcl::copyPointCloud (*featureCloudTargetTemp, *featureCloudTarget);

  // here is a guess transform that was manually set to align point clouds, pure icp performs well with this
  PointCloudNormal Final;
  Eigen::Matrix4f guess;
  guess <<   1, 0, 0, 0.07,
		     0, 1, 0, 0,
		     0, 0, 1, 0.015,
		     0, 0, 0, 1;

  ROS_INFO("Setting up icp with features");
  // custom icp
  pcl::IterativeClosestPointFeatures<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp_features;

  icp_features.setMaximumIterations (20);
  icp_features.setTransformationEpsilon (0);
  icp_features.setMaxCorrespondenceDistance(0.05);
  icp_features.setRANSACOutlierRejectionThreshold(0.05);

  icp_features.setInputCloud(cloud_source_normals);
  icp_features.setInputTarget(cloud_target_normals);
  icp_features.setSourceFeatures (featureCloudSource, indicesSource);
  icp_features.setTargetFeatures (featureCloudTarget, indicesTarget);
  icp_features.setFeatureErrorWeight(1);  // 1 = feature, 0 = icp

  ROS_INFO("Performing rgbd icp.....");
  icp_features.align(Final);  //, guess
  std::cout << "ICP features has finished with converge flag of:" << icp_features.hasConverged() << " score: " <<
		  icp_features.getFitnessScore() << std::endl;
  std::cout << icp_features.getFinalTransformation() << std::endl;


/*------BEST-------------
icp.getMaximumIterations 50
icp.getRANSACOutlierRejectionThreshold() 0.02
icp.getMaxCorrespondenceDistance() 0.03
icp.getTransformationEpsilon () 1e-09
icp.getEuclideanFitnessEpsilon () -1.79769e+308
score: 0.000164332
  ---------------------*/
  //Normal (non modified) icp for reference
  pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
    icp.setMaximumIterations (20);
    std::cerr << "icp.getMaximumIterations " << icp.getMaximumIterations() << std::endl;

    icp.setRANSACOutlierRejectionThreshold(0.05);
    std::cerr << "icp.getRANSACOutlierRejectionThreshold() " << icp.getRANSACOutlierRejectionThreshold() << std::endl;

    icp.setMaxCorrespondenceDistance(0.05);
    std::cerr << "icp.getMaxCorrespondenceDistance() " << icp.getMaxCorrespondenceDistance() << std::endl;

    //only used for convergence test
    icp.setTransformationEpsilon (0);
    std::cerr << "icp.getTransformationEpsilon () " << icp.getTransformationEpsilon () << std::endl;

    //only used for convergence test
    std::cerr << "icp.getEuclideanFitnessEpsilon () " << icp.getEuclideanFitnessEpsilon () << std::endl;

    icp.setInputCloud(featureCloudSource);
    icp.setInputTarget(featureCloudTarget);
    pcl::PointCloud<pcl::PointXYZRGBNormal> Final_reference;

    std::cout << "ICP has starts with a score of" << icp.getFitnessScore() << std::endl;

    ROS_INFO("Performing standard icp.....");
    icp.align(Final_reference);//, guess);
    std::cout << "ICP has finished with converge flag of:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
      std::cout << icp.getFinalTransformation() << std::endl;

  ROS_INFO("Writing output clouds...");
  transformPointCloud (*featureCloudSourceTemp, *cloud_converg_sparse, icp_features.getFinalTransformation());
  transformPointCloud (*cloud_source, *cloud_converg_dense, icp_features.getFinalTransformation());
  pcl::PCDWriter writer;
  writer.write ("cloud1-out.pcd", *cloud_source, false);
  writer.write ("cloud2-out.pcd", *cloud_target, false);
  writer.write ("normals-source.pcd", *cloud_source_normals, false);
  writer.write ("normals-target.pcd", *cloud_source_normals, false);
  writer.write ("feature-source.pcd", *featureCloudSource, false);
  writer.write ("feature-target.pcd", *featureCloudTarget, false);
  writer.write ("converged-cloud.pcd", *cloud_converg_dense, false);
  writer.write ("converged-feature.pcd", *cloud_converg_sparse, false);
  writer.write ("converged-reference.pcd", Final_reference, false);

 return (0);
}


