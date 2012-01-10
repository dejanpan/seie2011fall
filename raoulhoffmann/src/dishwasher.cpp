#include <iostream>
#include <cstdlib>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include "pcl_ros/io/bag_io.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZ PointT;

ros::Publisher pub;

Eigen::Vector4f min_pt, max_pt;

//min_pt = Eigen::Vector4f(0.5,0.16,0.63, 1);  // für rosbag 2011.12.06.13.xxx
//max_pt = Eigen::Vector4f(0.66,0.27,0.72, 1);

//min_pt = Eigen::Vector4f(0.2,0.2,0.2, 1);  // für rosbag 2011.12.06.13.xxx
//max_pt = Eigen::Vector4f(0.8,0.8,0.8, 1);  //

//min_pt = Eigen::Vector4f(0.4,0.4,0.4, 1);  // für rosbag 2011.12.06.13.xxx
//max_pt = Eigen::Vector4f(0.7,0.6,0.6, 1);  //

void cloudThrottledCallback(const sensor_msgs::PointCloud2::ConstPtr& inputCloud)
{
ROS_INFO("Message arrived");
sensor_msgs::PointCloud2 outputCloud;



boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );
//std::vector<int>& indices;
//std::vector<int> indices;
//pcl::PointIndices::Ptr indices (new pcl::PointIndices);



pcl::PointCloud<pcl::PointXYZ>::Ptr pclInputCloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pclOutputCloud (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

pcl::NormalEstimation<PointT, pcl::Normal> normalEstimator;

pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

//pcl::KdTree<PointT>::Ptr tree (new pcl::KdTree<PointT> ());


pcl::fromROSMsg(*inputCloud, *pclInputCloud);

//pcl::getPointsInBox(*inputCloud,min_pt,max_pt,*indices);
pcl::getPointsInBox(*pclInputCloud, min_pt,max_pt, *indices);   //for use with shared_pts

pcl::ExtractIndices<pcl::PointXYZ> ei;
ei.setInputCloud(pclInputCloud);
ei.setIndices(indices);
std::cout<<"Indices:";
std::cout<<indices->size()<<std::endl;
ei.filter(*cloud_filtered);

normalEstimator.setSearchMethod (tree);
normalEstimator.setInputCloud (cloud_filtered);
normalEstimator.setKSearch (50);
normalEstimator.compute (*cloud_normals);







//outputCloud = *inputCloud;

pcl::toROSMsg(*pclOutputCloud, outputCloud);

pub.publish(outputCloud);
}

int main(int argc, char **argv)
 {
     if(argc==7)
     {
        min_pt = Eigen::Vector4f(atof(argv[1]), atof(argv[2]), atof(argv[3]), 1);  // für rosbag 2011.12.06.13.xxx
        max_pt = Eigen::Vector4f(atof(argv[4]), atof(argv[5]), atof(argv[6]), 1);
     }



 ros::init(argc, argv, "dishwasher");
 ros::NodeHandle n;
 ros::Subscriber sub = n.subscribe("/kinect/cloud_throttled", 1000, cloudThrottledCallback);
 pub = n.advertise<sensor_msgs::PointCloud2> ("dishwasher_output", 1);
 ros::spin();
 }
