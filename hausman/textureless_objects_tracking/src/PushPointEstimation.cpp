/*
 * PushPointEstimation.cpp
 *
 *  Created on: Jul 16, 2012
 *      Author: Karol Hausman
 */

#include "textureless_objects_tracking/PushPointEstimation.h"

PushPointEstimation::PushPointEstimation() {
	label_rectangular_.reset(new pcl::PointCloud<pcl::PointXYZLRegion>);
	label_circular_.reset(new pcl::PointCloud<pcl::PointXYZLRegion>);
	label_other_.reset(new pcl::PointCloud<pcl::PointXYZLRegion>);

	euclidian_cluster_tolerance_=0.03;
	euclidian_min_cluster_size_=30;
	euclidian_max_cluster_size_=50000;

	max_distance_from_corner_concave_=0.02;
}

PushPointEstimation::~PushPointEstimation() {
	// TODO Auto-generated destructor stub
}

inline double getPoissonProbability (double l, int x)
{
  return pow(l,(double)x)*exp(-l)/boost::math::factorial<double>(x);
}

void PushPointEstimation::setAll3DCornersFromService(pcl::PointCloud<pcl::PointXYZLRegion>& concave_cloud,pcl::PointCloud<pcl::PointXYZLRegion>& convex_cloud){

	pcl::copyPointCloud(concave_cloud,concave_cloud_);
	pcl::copyPointCloud(convex_cloud,convex_cloud_);

}


void PushPointEstimation::getPushCloud(pcl::PointCloud<pcl::PointXYZLRegion>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZLRegion>& push_point_cloud,pcl::PointCloud<pcl::PointXYZLRegion> &push_point){

	getLabels(cloud);
	std::cout<<"size of the cloud: "<<cloud->size()<<std::endl;
	std::cout<<"size of the label_rect cloud: "<<label_rectangular_->size()<<std::endl;

	std::vector<pcl::PointCloud<pcl::PointXYZLRegion> > cluster_pointclouds_flat;
	std::vector<pcl::PointCloud<pcl::PointXYZLRegion> > cluster_pointclouds_round;
	std::vector<pcl::PointCloud<pcl::PointXYZLRegion> > cluster_pointclouds_other;

	euclidianClustering(label_rectangular_,cluster_pointclouds_flat);
	euclidianClustering(label_circular_,cluster_pointclouds_round);
	euclidianClustering(label_other_,cluster_pointclouds_other);


	std::cout<<"how many clusters flat: "<<cluster_pointclouds_flat.size()<<std::endl;
	std::cout<<"how many clusters round: "<<cluster_pointclouds_round.size()<<std::endl;
	std::cout<<"how many clusters other: "<<cluster_pointclouds_other.size()<<std::endl;

	pcl::PointCloud<pcl::PointXYZLRegion> uncertain_cloud_flat;
	std::string what="flat";
	double min_probability_flat=10000;
	if(cluster_pointclouds_flat.size()>0)
		getUncertainRegion(cluster_pointclouds_flat,uncertain_cloud_flat,what,min_probability_flat);
	std::cout<<"size of the most uncertain flat point cloud: "<<cluster_pointclouds_flat.size()<<std::endl;


	pcl::PointCloud<pcl::PointXYZLRegion> uncertain_cloud_round;
	what="round";
	double min_probability_round=10000;
	if(cluster_pointclouds_round.size()>0)
		getUncertainRegion(cluster_pointclouds_round,uncertain_cloud_round,what,min_probability_round);



	pcl::PointCloud<pcl::PointXYZLRegion> uncertain_cloud_other;
	what="other";
	double min_probability_other=10000;
	if(cluster_pointclouds_other.size()>0)
		getUncertainRegion(cluster_pointclouds_other,uncertain_cloud_other,what,min_probability_other);
//
//
	pcl::PointCloud<pcl::PointXYZLRegion> most_uncertain_cloud;
	if(min_probability_flat<min_probability_other){
		if(min_probability_flat<min_probability_round){
			pcl::copyPointCloud(uncertain_cloud_flat,most_uncertain_cloud);
		}else
			pcl::copyPointCloud(uncertain_cloud_round,most_uncertain_cloud);
	}else{
		if(min_probability_other<min_probability_round){
			pcl::copyPointCloud(uncertain_cloud_other,most_uncertain_cloud);
		}else
			pcl::copyPointCloud(uncertain_cloud_round,most_uncertain_cloud);
	}
	std::cout<<"min probability: "<<min_probability_flat<<std::endl;
	std::cout<<"size of the most uncertain point cloud: "<<most_uncertain_cloud.size()<<std::endl;
//
//
	pcl::copyPointCloud(most_uncertain_cloud,push_point_cloud);
	//TODO add the corners
	getPushPoint(push_point_cloud.makeShared(),push_point);

}

void PushPointEstimation::getPushPoint(const pcl::PointCloud<pcl::PointXYZLRegion>::Ptr& push_cloud,pcl::PointCloud<pcl::PointXYZLRegion>& push_point_cloud){

	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	pcl::PointCloud<pcl::PointXYZLRegion> search_cloud;


	pcl::search::KdTree<pcl::PointXYZLRegion>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZLRegion>);
	tree->setInputCloud(push_cloud);
//	pcl::PointCloud<pcl::PointXYZLRegion>& push_points_concave;
	int K=1;

//	if(concave_cloud_.size()>0)
//		pcl::copyPointCloud(concave_cloud_,search_cloud);
//	else
//		pcl::copyPointCloud(convex_cloud_,search_cloud);

	pcl::copyPointCloud(concave_cloud_,search_cloud);

	for (uint i=0;i<convex_cloud_.size();i++){

		search_cloud.push_back(convex_cloud_.points[i]);
	}

	for (uint i = 0; i < search_cloud.points.size(); i++) {

		pcl::PointXYZLRegion searchPoint = search_cloud.points[i];

		pointIdxNKNSearch.resize(1);
		pointNKNSquaredDistance.reserve(1);

		if (tree->nearestKSearch(searchPoint, K, pointIdxNKNSearch,
				pointNKNSquaredDistance) > 0) {
			for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j) {
				if ((pointIdxNKNSearch[j] >= 0)
						&& (pointNKNSquaredDistance[j] < max_distance_from_corner_concave_)) {
					push_point_cloud.push_back(
							searchPoint);
				}
			}

		}

	if(push_point_cloud.size()>0)
		break;
	}




}

void PushPointEstimation::getUncertainRegion(std::vector<pcl::PointCloud<pcl::PointXYZLRegion> >& cluster_pointclouds,pcl::PointCloud<pcl::PointXYZLRegion> &uncertain_cloud,std::string &what,double &min_probability){

	std::vector<size_t> sizes;
	std::vector<double> probabilities;
	for(uint i=0;i<cluster_pointclouds.size();i++){

		std::vector<uint32_t> vec;


		for (size_t j = 0; j < cluster_pointclouds[i].points.size(); j++) {
			uint32_t region=cluster_pointclouds[i].points[j].reg;
			vec.push_back(region);
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

	if(probabilities.size()>0){

		min_probability=probabilities[min_index];
		uncertain_cloud=cluster_pointclouds[min_index];
	}

}



void PushPointEstimation::euclidianClustering(
		pcl::PointCloud<pcl::PointXYZLRegion>::Ptr& cloudForEuclidianDistance,
		std::vector<pcl::PointCloud<pcl::PointXYZLRegion> >& cluster_pointclouds) {

	if ((cloudForEuclidianDistance!=NULL)&&(cloudForEuclidianDistance->size()>0)){


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

		if(cluster_indices.size()>0){

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
