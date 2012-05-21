#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

void normalEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudIn, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloudOut)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud (pointCloudIn);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.05);
  ne.compute (*pointCloudOut);
  pcl::copyPointCloud (*pointCloudIn, *pointCloudOut);
}

int
main (int argc, char** argv)
{
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSource (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTarget (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSourceFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTargetFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudSourceNormals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudTargetNormals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	reader.read (argv[1], *cloudSource);
	reader.read (argv[2], *cloudTarget);

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloudSource, *cloudSourceFiltered, indices);
	pcl::removeNaNFromPointCloud(*cloudTarget, *cloudTargetFiltered, indices);

	// calculate normals
	std::cout << "Calculating Normals..." << std::endl;
	normalEstimation(cloudSourceFiltered, cloudSourceNormals);
	normalEstimation(cloudTargetFiltered, cloudTargetNormals);

	// create icp object
	pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
	//create point to plane transformationEstimation object
	boost::shared_ptr< pcl::registration::TransformationEstimationLM<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal> >
		transformEstPointToPlane(new pcl::registration::TransformationEstimationLM<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal>());
	icp.setTransformationEstimation (transformEstPointToPlane);
	icp.setMaximumIterations (40);
	icp.setTransformationEpsilon (0);
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setRANSACOutlierRejectionThreshold(5);
	icp.setEuclideanFitnessEpsilon (0);

	  Eigen::Matrix4f guess;
	  guess << 1,   0,  0,  0.1,
			   0,	1,	0,	0.6,
			   0,	0,	1,	0.3,
			   0,	0,	0,	1;
	//  guess <<	0.998083, -0.0236136,  -0.057204, 0.068627,
	//			0.017266,    0.99389,   -0.10902, 0.627856,
	//			0.0594288,   0.107824,   0.992392,   0.370651,
	//			0,          0,          0,          1;
	//  guess  << 0.996215, 0.00456522, -0.0868249,   0.122714,
	//-0.0134495,   0.994693, -0.102021,   0.614769,
	// 0.0858986,   0.102803,   0.990987,   0.349081,
	//         0,        0,       0,        1;

	icp.setInputCloud(cloudSourceNormals);
	icp.setInputTarget(cloudTargetNormals);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_transformed( new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	std::cout << "Run ICP with point to plane error metric" << std::endl;
	icp.align(*cloud_transformed, guess);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out( new pcl::PointCloud<pcl::PointXYZRGB>);
	transformPointCloud (*cloudSource, *cloud_out,  icp.getFinalTransformation());
	//transformPointCloud (*cloudSource, *cloud_out,  guess);

	pcl::PCDWriter writer;
	writer.write ("output.pcd", *cloud_out, false);
  return (0);
}
