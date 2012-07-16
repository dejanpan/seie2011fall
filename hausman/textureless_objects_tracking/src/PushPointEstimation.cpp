/*
 * PushPointEstimation.cpp
 *
 *  Created on: Jul 16, 2012
 *      Author: Karol Hausman
 */

#include "textureless_objects_tracking/PushPointEstimation.h"

PushPointEstimation::PushPointEstimation() {
//	label_rectangular_=new (pcl::PointCloud<pcl::PointXYZLRegion>);
//	label_circular_=new (pcl::PointCloud<pcl::PointXYZLRegion>);
//	label_other_=new (pcl::PointCloud<pcl::PointXYZLRegion>);

}

PushPointEstimation::~PushPointEstimation() {
	// TODO Auto-generated destructor stub
}

inline double getPoissonProbability (double l, int x)
{
  return pow(l,(double)x)*exp(-l)/boost::math::factorial<double>(x);
}

void PushPointEstimation::getUncertainRegion(std::vector<pcl::PointCloud<pcl::PointXYZLRegion> >& cluster_pointclouds,pcl::PointCloud<pcl::PointXYZLRegion> &uncertain_cloud,std::string &what,double &min_probability){

	std::vector<size_t> sizes;
	std::vector<double> probabilities;
	for(uint i=0;i<cluster_pointclouds.size();i++){

		std::vector<uint32_t> vec;


		for (size_t j = 0; j < cluster_pointclouds[i].points.size(); j++) {
			uint32_t label=cluster_pointclouds[i].points[j].label;
			vec.push_back(label);
		}
		sort( vec.begin(), vec.end() );
		vec.erase( unique( vec.begin(), vec.end() ), vec.end() );
		sizes.push_back(vec.size());
	}

	for(uint number=0;number<sizes.size();number++){
		double l=0;
		if (what=="flat") l=1.87617;
		else if (what=="round") l=3.16577;
		else l=4.31672;

		probabilities.push_back(getPoissonProbability(l,sizes[number]));
	}
	int min_index=0;
	for(uint i=0;i<probabilities.size();i++){
		if(probabilities[i]<probabilities[min_index])
			min_index=i;
	}

	min_probability=probabilities[min_index];
	uncertain_cloud=cluster_pointclouds[min_index];

}



void PushPointEstimation::euclidianClustering(
		pcl::PointCloud<pcl::PointXYZLRegion>::Ptr& cloudForEuclidianDistance,
		std::vector<pcl::PointCloud<pcl::PointXYZLRegion> >& cluster_pointclouds) {


	std::vector<pcl::PointIndices> cluster_indices;
	pcl::search::KdTree<pcl::PointXYZLRegion>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZLRegion>);
	tree->setInputCloud(cloudForEuclidianDistance);

	pcl::EuclideanClusterExtraction<pcl::PointXYZLRegion> ec;
	ec.setClusterTolerance(euclidian_cluster_tolerance_);
	ec.setMinClusterSize(euclidian_min_cluster_size_);
	ec.setMaxClusterSize(euclidian_max_cluster_size_);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloudForEuclidianDistance);
	ec.extract(cluster_indices);


	for(uint i=0;i<cluster_indices.size();i++){
		pcl::PointCloud<pcl::PointXYZLRegion> cluster_pcl;
		for (size_t j = 0; j < cluster_indices[i].indices.size(); j++) {
			pcl::PointXYZLRegion point =
					cloudForEuclidianDistance->points[cluster_indices[i].indices[j]];
			cluster_pcl.points.push_back(point);
		}

		cluster_pointclouds.push_back(cluster_pcl);
	}
}


void PushPointEstimation::getLabels(pcl::PointCloud<pcl::PointXYZLRegion>::Ptr &cloud){

	for (size_t i = 0; i < cloud->points.size(); i++) {
							uint32_t label=cloud->points[i].label;

							if((label==1)||(label==4)){
								label_circular_->push_back(cloud->points[i]);
							}else if ((label==2)||(label==3)||(label==5)){
								label_rectangular_->push_back(cloud->points[i]);
							}else{
								label_other_->push_back(cloud->points[i]);
							}
						}


}
