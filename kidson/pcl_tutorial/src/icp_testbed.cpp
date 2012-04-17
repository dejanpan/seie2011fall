#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_features.h>
#include <pcl/features/normal_3d.h>
#include "pcl_ros/transforms.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/registration/transformation_estimation_wdf.h>

//rosbag stuff:
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "pcl_tutorial/featureMatch.h"
#include "pcl_tutorial/match.h"

#include <sstream>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;

#define MINIMUM_FEATURES	8

void normalEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudIn, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloudOut)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud (pointCloudIn);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.02); //0.02
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

void getTestDataFromBag(std::string bagfilename, PointCloudPtr cloud_source, PointCloudPtr cloud_target,
		PointCloudPtr featureCloudSource, std::vector<int> &indicesSource,
		PointCloudPtr featureCloudTarget, std::vector<int> &indicesTarget,
		Eigen::Matrix4f &initialTransform, int rosMessageNumber)
{
	sensor_msgs::PointCloud2::Ptr cloud_source_filtered (new sensor_msgs::PointCloud2 ());
	sensor_msgs::PointCloud2::Ptr cloud_target_filtered (new sensor_msgs::PointCloud2 ());

  	pcl::PCDWriter writer;
  	std::stringstream filename;
  	filename << "/home/ross/ros_workspace/bagfiles/scenesAllMatches/" << bagfilename << ".bag";

	rosbag::Bag bag;
	bag.open(filename.str(), rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(std::string("/feature_match_out_topic"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	ROS_INFO("opening bag time");
	int i = 1;
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		if(( i == rosMessageNumber) || (rosMessageNumber==0))
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
		    //int j = 1;
		  	for(std::vector<pcl_tutorial::match>::const_iterator iterator_ = fm->matches.begin(); iterator_ != fm->matches.end(); ++iterator_)
		    {
		  		indicesSource.push_back(iterator_->trainId);
		  		indicesTarget.push_back(iterator_->queryId);
		  	//	ROS_INFO_STREAM("source point " << j << ": "   << featureCloudSource->points[iterator_->queryId].x << ", " << featureCloudSource->points[iterator_->queryId].y << ", " << featureCloudSource->points[iterator_->queryId].z);
		  	//	ROS_INFO_STREAM("target point " << j++ << ": " << featureCloudTarget->points[iterator_->trainId].x << ", " << featureCloudTarget->points[iterator_->trainId].y << ", " << featureCloudTarget->points[iterator_->trainId].z);
		  	//	ROS_INFO("qidx: %d tidx: %d iidx: %d dist: %f", iterator_->queryId, iterator_->trainId, iterator_->imgId, iterator_->distance);
		  	}
		    /*for (size_t cloudId = 0; cloudId < featureCloudSource->points.size (); ++cloudId)
		    {
		    	ROS_INFO_STREAM("feature cloud: " << cloudId << ": " << featureCloudSource->points[cloudId].x << "; " << featureCloudSource->points[cloudId].y << "; " << featureCloudSource->points[cloudId].z);
		    }*/
/*
		    ROS_INFO("Writing point clouds");
		  	filename << "cloud" << i << "DenseSource.pcd";
		  	writer.write (filename.str(), *cloud_source, false);
		  	filename.str("");
		  	filename << "cloud" << i << "DenseTarget.pcd";
		  	writer.write (filename.str(), *cloud_target, false);
		  	filename.str("");
		  	filename << "cloud" << i << "SparseSource.pcd";
		  	writer.write (filename.str(), *featureCloudSource, false);
		  	filename.str("");
		  	filename << "cloud" << i << "SparseTarget.pcd";
		    writer.write (filename.str(), *featureCloudTarget, false);
		    filename.str("");*/
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
/*
void removeCorrespondancesZThreshold (PointCloudPtr featureCloudSource, std::vector<int> &indicesSource,
		PointCloudPtr featureCloudTarget, std::vector<int> &indicesTarget,
		float zThreshold)
{
    for (size_t corrIdx = 0; corrIdx < indicesSource.size (); corrIdx++)
    {
        if(featureCloudSource->points[indicesSource[corrIdx]].z > zThreshold)
        {
            //remove from source and target indices
            indicesSource.erase(indicesSource.begin() + (int)corrIdx);
            indicesTarget.erase(indicesTarget.begin() + (int)corrIdx);
            corrIdx--;  //because indices are now - 1
        }
    }
}
*/

double calculateOverlap(PointCloudPtr cloudSource, PointCloudPtr cloudTarget)
{
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(cloudTarget);
  double max_range = 0.0005;

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  int nr = 0;
  for (size_t i = 0; i < cloudSource->points.size (); ++i)
  {
	kdtree.nearestKSearch (cloudSource->points[i], 1, nn_indices, nn_dists);
    if (nn_dists[0] <= max_range)
      	nr++;
  }
  return (nr / (double)cloudSource->points.size());
}

float runICPTest (std::string bagfilename, int messageIdx, float alpha, std::string cloudDist)
{
	PointCloudPtr cloud_source (new PointCloud);
	PointCloudPtr cloud_target (new PointCloud);
	PointCloudPtr featureCloudSourceTemp (new PointCloud);
	PointCloudPtr featureCloudTargetTemp (new PointCloud);
	PointCloudPtr cloud_converg_sparse_all (new PointCloud);
	PointCloudPtr cloud_converg_sparse_correspond (new PointCloud);
	PointCloudPtr cloud_target_sparse_correspond (new PointCloud);
	PointCloudPtr cloud_converg_dense (new PointCloud);
	PointCloudPtr cloud_ransac_estimation (new PointCloud);
	PointCloudNormalPtr cloud_source_normals (new PointCloudNormal);
	PointCloudNormalPtr cloud_target_normals (new PointCloudNormal);
	PointCloudNormalPtr featureCloudSource (new PointCloudNormal);
	PointCloudNormalPtr featureCloudTarget (new PointCloudNormal);
	Eigen::Matrix4f initialTransform;
	std::vector<int> indicesSource;
	std::vector<int> indicesTarget;

	ROS_INFO("Getting test data from a bag file");
	getTestDataFromBag(bagfilename, cloud_source, cloud_target, featureCloudSourceTemp, indicesSource, featureCloudTargetTemp, indicesTarget, initialTransform, messageIdx);
	Eigen::Matrix4f ransacInverse = initialTransform.inverse();

	 //calculate normals
	ROS_INFO("Calcualting normals");
	normalEstimation(cloud_source, cloud_source_normals);
	normalEstimation(cloud_target, cloud_target_normals);

	ROS_INFO("Converting feature point clouds");
	pcl::copyPointCloud (*featureCloudSourceTemp, *featureCloudSource);
	pcl::copyPointCloud (*featureCloudTargetTemp, *featureCloudTarget);

	std::vector<int> indicesSourceSmall = indicesSource;
	std::vector<int> indicesTargetSmall = indicesTarget;

	//add cloud size to feature correspondences
	for(std::vector<int>::iterator iterator_ = indicesSource.begin(); iterator_ != indicesSource.end(); ++iterator_) {
		*iterator_ = *iterator_ + cloud_source->size();
	}
	for(std::vector<int>::iterator iterator_ = indicesTarget.begin(); iterator_ != indicesTarget.end(); ++iterator_) {
		*iterator_ = *iterator_ + cloud_target->size();
	}

	//indicesSource.clear();
	//indicesTarget.clear();

	//add feature correspondances to dense clouds
	PointCloudNormalPtr concatinatedSourceCloud (new PointCloudNormal);
	PointCloudNormalPtr concatinatedTargetCloud (new PointCloudNormal);

	*concatinatedSourceCloud = *cloud_source_normals;
	*concatinatedTargetCloud = *cloud_target_normals;

	(*concatinatedSourceCloud) += *featureCloudSource;
	(*concatinatedTargetCloud) += *featureCloudTarget;

	boost::shared_ptr< TransformationEstimationWDF<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal> >
	  		initialTransformWDF(new TransformationEstimationWDF<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal>());

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
	ROS_INFO_STREAM("---------------------------------------------------------      indices size: " << indicesSource.size() );
	if(indicesSource.size() < MINIMUM_FEATURES)
		icp_wdf.align ( *cloud_transformed);
	else
		icp_wdf.align ( *cloud_transformed, ransacInverse);
	std::cout << "[SIIMCloudMatch::runICPMatch] Has converged? = " << icp_wdf.hasConverged() << std::endl <<
	"	fitness score (SSD): " << icp_wdf.getFitnessScore (1000) << std::endl;
	icp_wdf.getFinalTransformation ();

	ROS_INFO("Transforming clouds with obtained transform...");
	  transformPointCloud (*featureCloudSourceTemp, *cloud_converg_sparse_all, icp_wdf.getFinalTransformation());
	  transformPointCloud (*featureCloudSourceTemp, *cloud_converg_sparse_correspond, icp_wdf.getFinalTransformation());
	  transformPointCloud (*cloud_source, *cloud_converg_dense, icp_wdf.getFinalTransformation());
	  transformPointCloud (*cloud_source, *cloud_ransac_estimation, ransacInverse);

	  // remove non correspondences from feature clouds as an addition output
	  uint8_t col=3;
	  // *cloud_converg_sparse_correspond = *cloud_converg_sparse_all;
	  cloud_converg_sparse_correspond->points.resize(indicesSourceSmall.size());
	  cloud_converg_sparse_correspond->height = 1;
	  cloud_converg_sparse_correspond->width = (int)indicesSourceSmall.size();

	  cloud_target_sparse_correspond->points.resize(indicesTargetSmall.size());
	  cloud_target_sparse_correspond->height = 1;
	  cloud_target_sparse_correspond->width = (int)indicesTargetSmall.size();
	  for (size_t cloudId = 0; cloudId < indicesSourceSmall.size(); ++cloudId)
	  {
		  cloud_converg_sparse_correspond->points[cloudId].x = cloud_converg_sparse_all->points[indicesSourceSmall[cloudId]].x;
		  cloud_converg_sparse_correspond->points[cloudId].y = cloud_converg_sparse_all->points[indicesSourceSmall[cloudId]].y;
		  cloud_converg_sparse_correspond->points[cloudId].z = cloud_converg_sparse_all->points[indicesSourceSmall[cloudId]].z;
		  cloud_converg_sparse_correspond->points[cloudId].r = col;
		  cloud_converg_sparse_correspond->points[cloudId].g = col;
		  cloud_converg_sparse_correspond->points[cloudId].b = col;
		  cloud_target_sparse_correspond->points[cloudId].x = featureCloudTarget->points[indicesTargetSmall[cloudId]].x;
		  cloud_target_sparse_correspond->points[cloudId].y = featureCloudTarget->points[indicesTargetSmall[cloudId]].y;
		  cloud_target_sparse_correspond->points[cloudId].z = featureCloudTarget->points[indicesTargetSmall[cloudId]].z;
		  cloud_target_sparse_correspond->points[cloudId].r = col;
		  cloud_target_sparse_correspond->points[cloudId].g = col;
		  cloud_target_sparse_correspond->points[cloudId].b = col;
		  //std::cerr << "point " << cloudId << "\n ";
	  }

	  ROS_INFO("Writing output clouds...");
	  pcl::PCDWriter writer;
	  std::stringstream filename;

	  filename << bagfilename << "-" << cloudDist << "-" << "alpha" << alpha << "-" << messageIdx << "-targetDense.pcd";
	  writer.write (filename.str(), *cloud_target, false);
	  filename.str("");
	  filename << bagfilename << "-" << cloudDist << "-" << "alpha" << alpha << "-" << messageIdx << "-convergedDense.pcd";
	  writer.write (filename.str(), *cloud_converg_dense, false);
	  filename.str("");
	  filename << bagfilename << "-" << cloudDist << "-" << "alpha" << alpha << "-" << messageIdx << "-targetSparse.pcd";
	  writer.write (filename.str(), *cloud_target_sparse_correspond, false);
	  filename.str("");
	  filename << bagfilename << "-" << cloudDist << "-" << "alpha" << alpha << "-" << messageIdx << "-convergedSparse.pcd";
	  writer.write (filename.str(), *cloud_converg_sparse_correspond, false);
	  filename.str("");

	  std::cout << "##################### OVERLAP ###################: " << calculateOverlap(cloud_target, cloud_ransac_estimation) << "\n";

	  return icp_wdf.getFitnessScore (0.05);

}

float runICPSVDTest (std::string bagfilename, int messageIdx, std::string cloudDist)
{
	PointCloudPtr cloud_source (new PointCloud);
	PointCloudPtr cloud_target (new PointCloud);
	PointCloudPtr featureCloudSourceTemp (new PointCloud);
	PointCloudPtr featureCloudTargetTemp (new PointCloud);
	Eigen::Matrix4f initialTransform;
	std::vector<int> indicesSource;
	std::vector<int> indicesTarget;

	ROS_INFO("Getting test data from a bag file");
	getTestDataFromBag(bagfilename, cloud_source, cloud_target, featureCloudSourceTemp, indicesSource, featureCloudTargetTemp, indicesTarget, initialTransform, messageIdx);
	Eigen::Matrix4f ransacInverse = initialTransform.inverse();

	 //Normal (non modified) icp for reference
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setMaximumIterations (20);
	std::cerr << "icp.getMaximumIterations " << icp.getMaximumIterations() << std::endl;
	icp.setRANSACOutlierRejectionThreshold(0.05);
	std::cerr << "icp.getRANSACOutlierRejectionThreshold() " << icp.getRANSACOutlierRejectionThreshold() << std::endl;
	icp.setMaxCorrespondenceDistance(0.05);
	std::cerr << "icp.getMaxCorrespondenceDistance() " << icp.getMaxCorrespondenceDistance() << std::endl;
	icp.setTransformationEpsilon (0);
	std::cerr << "icp.getTransformationEpsilon () " << icp.getTransformationEpsilon () << std::endl;
	//only used for convergence test
	std::cerr << "icp.getEuclideanFitnessEpsilon () " << icp.getEuclideanFitnessEpsilon () << std::endl;

	icp.setInputCloud(cloud_source);
	icp.setInputTarget(cloud_target);
	pcl::PointCloud<pcl::PointXYZRGB> Final_reference;

	std::cout << "ICP has starts with a score of" << icp.getFitnessScore() << std::endl;

	ROS_INFO("Performing standard icp.....");
	ROS_INFO_STREAM("---------------------------------------------------------      indices size: " << indicesSource.size() );
	if(indicesSource.size() < MINIMUM_FEATURES)
		icp.align(Final_reference);  // don't use ransac for small number of correspondences
	else
		icp.align(Final_reference, ransacInverse);
	std::cout << "ICP has finished with converge flag of:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;


	ROS_INFO("Writing output clouds...");
	pcl::PCDWriter writer;
	std::stringstream filename;

	filename << bagfilename << "-" << cloudDist << "-" << "SVD" << "-targetDense.pcd";
	writer.write (filename.str(), *cloud_target, false);
	filename.str("");
	filename << bagfilename << "-" << cloudDist << "-" << "SVD" << "-convergedDense.pcd";
	writer.write (filename.str(), Final_reference, false);
	filename.str("");

	return icp.getFitnessScore (0.05);
}

void runTests(std::vector<double>& results, std::string filename, int id, std::string cloudDist)
{
	std::cout << "############################################################################################# \n";
	std::cout << "starting test for: " << filename << ", using alpha: 0.0 and cloud separation of: " << cloudDist << "\n";
	results.push_back(runICPTest(filename, id, 0.0, cloudDist));
	std::cout << "############################################################################################# \n";
	std::cout << "starting test for: " << filename << " id: " << id << ", using alpha: 0.5 and cloud separation of: " << cloudDist << "\n";
	results.push_back(runICPTest(filename, id, 0.5, cloudDist));
	std::cout << "############################################################################################# \n";
	std::cout << "starting test for: " << filename << ", using alpha: 1.0 and cloud separation of: " << cloudDist << "\n";
	results.push_back(runICPTest(filename, id, 1.0, cloudDist));
	std::cout << "############################################################################################# \n";
	std::cout << "starting test for: " << filename << ", using SVD and cloud separation of: " << cloudDist << "\n";
	results.push_back(runICPSVDTest(filename, id, cloudDist));
}


int main (int argc, char** argv)
{
	std::vector<double> results;

	PointCloudPtr cloudSource (new PointCloud);
	PointCloudPtr cloudTarget (new PointCloud);

	/*pcl::PCDReader reader;
	reader.read (argv[1], *cloudSource);
	reader.read (argv[2], *cloudTarget);
	std::cout << "overlap:" << 100 * calculateOverlap(cloudSource, cloudTarget) << "% \n";*/

	/*runTests(results, "NoFeatureNonStructured1-features2", 1, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 2, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 3, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 4, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 5, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 6, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 7, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 8, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 9, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 10, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 11, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 12, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 13, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 14, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 15, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 16, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 17, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 18, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 19, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 20, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 21, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 22, "close"); //close
		runTests(results, "NoFeatureNonStructured1-features2", 23, "close"); //close*/


//	runTests(results, "bench1-2sweeps2", 8, "mid"); //close
//	runTests(results, "bench1-2sweeps2", 12, "far"); //close

/*	runTests(results, "featureStructured3-features", 63, "close"); //close
	runTests(results, "featureStructured3-features", 64, "mid"); //medr
	runTests(results, "featureStructured3-features", 62, "far"); //far

	runTests(results, "featureNonStructured3-features", 78, "close"); //close
	runTests(results, "featureNonStructured3-features", 80, "mid"); //med
	runTests(results, "featureNonStructured3-features", 79, "far"); //far

	runTests(results, "NoFeatureStructured3-features", 16, "close"); //close
	runTests(results, "NoFeatureStructured3-features", 18, "mid"); //med
	runTests(results, "NoFeatureStructured3-features", 17, "far"); //far

	runTests(results, "NoFeatureNonStructured1-features", 73, "close"); //close
	runTests(results, "NoFeatureNonStructured1-features", 74, "mid"); //med
	runTests(results, "NoFeatureNonStructured1-features", 75, "far"); //far

	runTests(results, "desk-close1-features", 50, "close"); //close
	runTests(results, "desk-close1-features", 52, "mid"); //med
	runTests(results, "desk-close1-features", 51, "far"); //far

	runTests(results, "desk-middle1-features", 37, "close"); //close
	runTests(results, "desk-middle1-features", 39, "mid"); //med
	runTests(results, "desk-middle1-features", 38, "far"); //far

	runTests(results, "desk-far1-features", 46, "close"); //close
	runTests(results, "desk-far1-features", 47, "mid"); //med
	runTests(results, "desk-far1-features", 48, "far"); //far*/

	runTests(results, "bench1-2sweeps5", 1, "close"); //close
	runTests(results, "bench1-2sweeps5", 17, "mid"); //medr
	runTests(results, "bench1-2sweeps5", 32, "far"); //far

	runTests(results, "featureNonStructured3-features", 78, "close"); //close
	runTests(results, "featureNonStructured3-features", 80, "mid"); //med
	runTests(results, "featureNonStructured3-features", 79, "far"); //far

	runTests(results, "NoFeatureStructured3-features", 16, "close"); //close
	runTests(results, "NoFeatureStructured3-features", 18, "mid"); //med
	runTests(results, "NoFeatureStructured3-features", 17, "far"); //far

	runTests(results, "NoFeatureNonStructured1-features2", 10, "close"); //close
	runTests(results, "NoFeatureNonStructured1-features2", 23, "mid"); //med
	runTests(results, "NoFeatureNonStructured1-features2", 20, "far"); //far

	runTests(results, "desk-close1-features3", 1, "close"); //close
	runTests(results, "desk-close1-features3", 13, "mid"); //med
	runTests(results, "desk-close1-features3", 17, "far"); //far

	runTests(results, "desk-middle1-features2", 1, "close"); //close
	runTests(results, "desk-middle1-features2", 4, "mid"); //med
	runTests(results, "desk-middle1-features2", 2, "far"); //far

	runTests(results, "desk-far1-features2", 1, "close"); //close
	runTests(results, "desk-far1-features2", 7, "mid"); //med
	runTests(results, "desk-far1-features2", 23, "far"); //far

/*	runTests(results, "bench1-2sweeps", 16, "close"); //close
	runTests(results, "bench1-2sweeps", 26, "close"); //close
	runTests(results, "bench1-2sweeps", 36, "close"); //close
	runTests(results, "bench1-2sweeps", 46, "close"); //close
	runTests(results, "bench1-2sweeps", 56, "close"); //close
	runTests(results, "bench1-2sweeps", 66, "close"); //close
	runTests(results, "bench1-2sweeps", 76, "close"); //close
	runTests(results, "bench1-2sweeps", 86, "close"); //close
	runTests(results, "bench1-2sweeps", 96, "close"); //close
	runTests(results, "bench1-2sweeps", 106, "close"); //close */

	for (size_t idx = 0; idx < results.size(); ++idx)
	{
		ROS_INFO_STREAM("Results " << idx << ": " << results[idx]);
	}

	return (0);
}

/*

int main (int argc, char** argv)
{
  PointCloudPtr cloud_source (new PointCloud);
  PointCloudPtr cloud_target (new PointCloud);
  PointCloudPtr featureCloudSourceTemp (new PointCloud);
  PointCloudPtr featureCloudTargetTemp (new PointCloud);
  PointCloudPtr cloud_converg_sparse_all (new PointCloud);
  PointCloudPtr cloud_converg_sparse_correspond (new PointCloud);
  PointCloudPtr cloud_target_sparse_correspond (new PointCloud);
  PointCloudPtr cloud_converg_dense (new PointCloud);
  PointCloudPtr cloud_ransac_estimation (new PointCloud);
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
  getTestDataFromBag("2012-03-30-21-55-54", cloud_source, cloud_target, featureCloudSourceTemp, indicesSource, featureCloudTargetTemp, indicesTarget, initialTransform, 0);
}
  // remove corresponances with large z values (susceptible to error)
  ROS_INFO("Removing feature correspondances with a large depth measurement");
  //removeCorrespondancesZThreshold (cloud_source, 1.8);

  pcl::PCDWriter writer;
  writer.write ("cloud1-out.pcd", *cloud_source, false);
  writer.write ("cloud2-out.pcd", *cloud_target, false);
 // writer.write ("converged-reference.pcd", Final_reference, false);

  //calculate normals
  ROS_INFO("Calcualting normals");
 normalEstimation(cloud_source, cloud_source_normals);
 normalEstimation(cloud_target, cloud_target_normals);

  ROS_INFO("Converting feature point clouds");
  pcl::copyPointCloud (*featureCloudSourceTemp, *featureCloudSource);
  pcl::copyPointCloud (*featureCloudTargetTemp, *featureCloudTarget);

  // here is a guess transform that was manually set to align point clouds, pure icp performs well with this
  PointCloud Final;
  Eigen::Matrix4f guess;
  guess <<   1, 0, 0, 0.07,
		     0, 1, 0, 0,
		     0, 0, 1, 0.015,
		     0, 0, 0, 1;

  ROS_INFO("Setting up icp with features");
   custom icp
  pcl::IterativeClosestPointFeatures<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp_features;
  icp_features.setMaximumIterations (40);
  icp_features.setTransformationEpsilon (1e-9);
  icp_features.setMaxCorrespondenceDistance(0.1);
  icp_features.setRANSACOutlierRejectionThreshold(0.03);

  icp_features.setInputCloud(cloud_source_normals);
  icp_features.setInputTarget(cloud_target_normals);
  icp_features.setSourceFeatures (featureCloudSource, indicesSource);
  icp_features.setTargetFeatures (featureCloudTarget, indicesTarget);
  icp_features.setFeatureErrorWeight(0.3);  // 1 = feature, 0 = icp

  ROS_INFO("Performing rgbd icp.....");
  icp_features.align(Final, ransacInverse);
  std::cout << "ICP features has finished with converge flag of:" << icp_features.hasConverged() << " score: " <<
		  icp_features.getFitnessScore() << std::endl;
  std::cout << icp_features.getFinalTransformation() << std::endl;

  std::vector<int> indicesSourceSmall = indicesSource;
  std::vector<int> indicesTargetSmall = indicesTarget;

  for(std::vector<int>::iterator iterator_ = indicesSource.begin(); iterator_ != indicesSource.end(); ++iterator_) {
  	*iterator_ = *iterator_ + cloud_source->size();
  }
  for(std::vector<int>::iterator iterator_ = indicesTarget.begin(); iterator_ != indicesTarget.end(); ++iterator_) {
      *iterator_ = *iterator_ + cloud_target->size();
  }

  PointCloudNormalPtr concatinatedSourceCloud (new PointCloudNormal);
  PointCloudNormalPtr concatinatedTargetCloud (new PointCloudNormal);

  *concatinatedSourceCloud = *cloud_source_normals;
  *concatinatedTargetCloud = *cloud_target_normals;

  (*concatinatedSourceCloud) += *featureCloudSource;
  (*concatinatedTargetCloud) += *featureCloudTarget;

  boost::shared_ptr< TransformationEstimationWDF<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal> >
  		initialTransformWDF(new TransformationEstimationWDF<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal>());

  float alpha = 0.5;
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
  icp_wdf.align ( *cloud_transformed);//, ransacInverse); //init_tr );
  std::cout << "[SIIMCloudMatch::runICPMatch] Has converged? = " << icp_wdf.hasConverged() << std::endl <<
				"	fitness score (SSD): " << icp_wdf.getFitnessScore (1000) << std::endl;
  icp_wdf.getFinalTransformation ();

  /// Final ICP transformation is obtained by multiplying initial transformed with icp refinement

------BEST-------------
icp.getMaximumIterations 50
icp.getRANSACOutlierRejectionThreshold() 0.02
icp.getMaxCorrespondenceDistance() 0.03
icp.getTransformationEpsilon () 1e-09
icp.getEuclideanFitnessEpsilon () -1.79769e+308
score: 0.000164332
  ---------------------
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
  transformPointCloud (*featureCloudSourceTemp, *cloud_converg_sparse_all, icp_wdf.getFinalTransformation());
  transformPointCloud (*featureCloudSourceTemp, *cloud_converg_sparse_correspond, icp_wdf.getFinalTransformation());
  transformPointCloud (*cloud_source, *cloud_converg_dense, icp_wdf.getFinalTransformation());
  transformPointCloud (*cloud_source, *cloud_ransac_estimation, ransacInverse);

  // remove non correspondences from feature clouds as an addition output
  uint8_t col=3;
  // *cloud_converg_sparse_correspond = *cloud_converg_sparse_all;
  cloud_converg_sparse_correspond->points.resize(indicesSourceSmall.size());
  cloud_converg_sparse_correspond->height = 1;
  cloud_converg_sparse_correspond->width = (int)indicesSourceSmall.size();

  cloud_target_sparse_correspond->points.resize(indicesTargetSmall.size());
  cloud_target_sparse_correspond->height = 1;
  cloud_target_sparse_correspond->width = (int)indicesTargetSmall.size();
  for (size_t cloudId = 0; cloudId < indicesSourceSmall.size(); ++cloudId)
  {
	  cloud_converg_sparse_correspond->points[cloudId].x = cloud_converg_sparse_all->points[indicesSourceSmall[cloudId]].x;
	  cloud_converg_sparse_correspond->points[cloudId].y = cloud_converg_sparse_all->points[indicesSourceSmall[cloudId]].y;
	  cloud_converg_sparse_correspond->points[cloudId].z = cloud_converg_sparse_all->points[indicesSourceSmall[cloudId]].z;
	  cloud_converg_sparse_correspond->points[cloudId].r = col;
	  cloud_converg_sparse_correspond->points[cloudId].g = col;
	  cloud_converg_sparse_correspond->points[cloudId].b = col;
	  cloud_target_sparse_correspond->points[cloudId].x = featureCloudTarget->points[indicesTargetSmall[cloudId]].x;
	  cloud_target_sparse_correspond->points[cloudId].y = featureCloudTarget->points[indicesTargetSmall[cloudId]].y;
	  cloud_target_sparse_correspond->points[cloudId].z = featureCloudTarget->points[indicesTargetSmall[cloudId]].z;
	  cloud_target_sparse_correspond->points[cloudId].r = col;
	  cloud_target_sparse_correspond->points[cloudId].g = col;
	  cloud_target_sparse_correspond->points[cloudId].b = col;
	  //std::cerr << "point " << cloudId << "\n ";
  }

  pcl::PCDWriter writer;
  writer.write ("cloud1-out.pcd", *cloud_source, false);
  writer.write ("cloud2-out.pcd", *cloud_target, false);
  writer.write ("normals-source.pcd", *cloud_source_normals, false);
  writer.write ("normals-target.pcd", *cloud_source_normals, false);
  writer.write ("feature-source.pcd", *featureCloudSource, false);
  writer.write ("feature-target.pcd", *featureCloudTarget, false);
  writer.write ("converged-cloud.pcd", *cloud_converg_dense, false);
  writer.write ("converged-feature-all.pcd", *cloud_converg_sparse_all, false);
  writer.write ("converged-feature-correspond.pcd", *cloud_converg_sparse_correspond, false);
  writer.write ("target-feature-correspond.pcd", *cloud_target_sparse_correspond, false);
  writer.write ("ransac_estimation.pcd", *cloud_ransac_estimation, false);
 // writer.write ("converged-reference.pcd", Final_reference, false);

 return (0);
}
*/
