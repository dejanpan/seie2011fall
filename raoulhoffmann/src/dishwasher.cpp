#include <iostream>
#include <cstdlib>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include <pcl/io/io.h>

#include "pcl_ros/io/bag_io.h"

#include "sensor_msgs/point_cloud_conversion.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZRGB PointT;

//use later for mark detected cups
ros::Publisher vis_pub;

ros::Publisher pubDebug;
ros::Publisher pubResult;

//tf TRY
//tf::TransformListener listener;

Eigen::Vector4f min_pt, max_pt;

bool useBox;

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

//tf::StampedTransform transform;
//listener.lookupTransform("/openni_rgb_optical_frame", "/map", ros::Time(0), transform);

boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );
//boost::shared_ptr<pcl::PointIndices> indices_segments (new pcl::PointIndices)
boost::shared_ptr<pcl::PointIndices> inliers_cylinder (new pcl::PointIndices);
//std::vector<int>& indices;
//std::vector<int> indices;
//pcl::PointIndices::Ptr indices (new pcl::PointIndices);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclInputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclOutputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

pcl::NormalEstimation<PointT, pcl::Normal> normalEstimator;

pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

//pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
//boost::shared_ptr<pcl::PointIndices> inliers_cylinder (new pcl::PointIndices);

pcl::fromROSMsg(*inputCloud, *pclInputCloud);

// DOWNSAMPLING - NECESSARY??
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr    cloud_filtered     (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr    oneCluster         (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr    colouredClusters   (new pcl::PointCloud<pcl::PointXYZRGB>);

  vg.setInputCloud (pclInputCloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*



// SEGMENT AND EXTRACT

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  //boost::shared_ptr<std::vector<int> > cluster_indices( new std::vector<int> );

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (50000);
  ec.setSearchMethod (tree);
  ec.setInputCloud( cloud_filtered);
  ec.extract (cluster_indices);


  //pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);

    pcl::ExtractIndices<pcl::PointXYZRGB> ei;

ei.setInputCloud(cloud_filtered);


pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

visualization_msgs::MarkerArray markerArray;

for (unsigned int curr_cluster = 0; curr_cluster < cluster_indices.size();  curr_cluster++)
{
    ei.setIndices(boost::make_shared<pcl::PointIndices>(cluster_indices[curr_cluster]));
    curr_cluster = curr_cluster + 1;
    ei.filter(*oneCluster);




    uint8_t r = static_cast<uint8_t>(rand() % 255 + 1);
    uint8_t g = static_cast<uint8_t>(rand() % 255 + 1);
    uint8_t b = static_cast<uint8_t>(rand() % 255 + 1);

    int32_t rgb = (r << 16) | (g << 8) | b;

    for(unsigned int z = 0; z< oneCluster->points.size(); z++)
    {
        oneCluster->points[z].rgb = *(float *)(&rgb);
    }

    pcl::concatenateFields (*colouredClusters, *oneCluster, *colouredClusters);


    normalEstimator.setSearchMethod (tree);
    normalEstimator.setInputCloud (oneCluster);
    normalEstimator.setKSearch (50);
    normalEstimator.compute (*cloud_normals);


// try cylinder segmentation

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  //cloud_filtered consists of points in box
  seg.setInputCloud (oneCluster);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);



visualization_msgs::Marker marker;
marker.header.frame_id = "/openni_rgb_optical_frame";
marker.header.stamp = ros::Time();
marker.type = visualization_msgs::Marker::CYLINDER;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = coefficients_cylinder->values[0];
marker.pose.position.y = coefficients_cylinder->values[1];
marker.pose.position.z = coefficients_cylinder->values[2];
marker.pose.orientation.x = coefficients_cylinder->values[3];
marker.pose.orientation.y = coefficients_cylinder->values[4];
marker.pose.orientation.z = coefficients_cylinder->values[5];
marker.pose.orientation.w = 1.0;
marker.scale.x = coefficients_cylinder->values[6];
marker.scale.y = coefficients_cylinder->values[6];
marker.scale.z = 0.1;
marker.color.a = 1.0;
marker.color.r = 1.0;
marker.color.g = 1.0;
marker.color.b = 0.0;


markerArray.markers.push_back(marker);


}

/*
for(int i=0; i< cluster_indices.size(); i++)
{
    pcl::ExtractIndices<pcl::PointXYZ> ei;

    ei.setInputCloud(cloud_filtered);
    ei.setIndices((*cluster_indices).at(i));
    ei.filter(*cloud_filtered);

}*/


  //for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  //{
    //pcl::PointCloud<pcl::PointXYZ>::Ptr one_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    //pcl::PointIndices::Ptr one_cluster;

    //boost::shared_ptr<std::vector<int> > one_cluster( new std::vector<int> );

    // for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    //   one_cluster->indices.push_back(it->indices[*pit]);


    /*
    pcl::ExtractIndices<pcl::PointXYZ> ei;

    ei.setInputCloud(cloud_filtered);
    ei.setIndices(it->indices);
    ei.filter(*cloud_filtered);
    */
  //}


//std::cout << "Segment Vector Entry 0: " << cluster_segments[0]->points.size () << " data points." << std::endl;

// PUBLISH CLOUDS
//pcl::toROSMsg(*cloud_cluster, outputCloud);


std::cout << "Number of Markers: " << markerArray.markers.size ()  << std::endl;


vis_pub.publish( markerArray );


pcl::toROSMsg(*colouredClusters, outputCloud);
pubDebug.publish(outputCloud);
pubResult.publish(outputCloud);

}

int main(int argc, char **argv)
 {
     if(argc==7)
     {
        min_pt = Eigen::Vector4f(atof(argv[1]), atof(argv[2]), atof(argv[3]), 1);  // für rosbag 2011.12.06.13.xxx
        max_pt = Eigen::Vector4f(atof(argv[4]), atof(argv[5]), atof(argv[6]), 1);
        useBox = true;
     }
     else
     {
        min_pt = Eigen::Vector4f(0, 0, 0, 1);  // für rosbag 2011.12.06.13.xxx
        max_pt = Eigen::Vector4f(1, 1, 1, 1);
        useBox = false;
     }




 ros::init(argc, argv, "dishwasher");
 ros::NodeHandle n;
 ros::Subscriber sub = n.subscribe("/kinect/cloud_throttled", 1000, cloudThrottledCallback);
 pubDebug = n.advertise<sensor_msgs::PointCloud2> ("dishwasher_debug", 1);
 pubResult = n.advertise<sensor_msgs::PointCloud2> ("dishwasher_result", 1);
// vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

 ros::spin();
 }
