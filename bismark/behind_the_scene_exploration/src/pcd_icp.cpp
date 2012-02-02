#include <math.h>
#include "ros/ros.h"
#include "simple_robot_control/robot_control.h"
#include "tf/transform_listener.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl_ros/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/registration/registration.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/icp_nl.h"

using namespace std;

//Constants
#define FILTER_MAX 1.5
#define FILTER_MIN 0
#define MAX_ITR 50
#define MAX_DIST 0.01
#define TRANS_EPSILON 1e-10



int main(int argc, char** argv){


	ros::init(argc, argv, "behind_the_scene_exploration");
	ros::NodeHandle n;

    bool first_concat = true;

    tf::Transform trans;
    tf::Quaternion axis;
    tf::TransformListener tf_;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr concat_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input  (new pcl::PointCloud<pcl::PointXYZRGB>);

    for(int i=220;i<=320;i+=20){

    	std::stringstream ss;
    	ss<<i;
    	pcl::PCDReader reader;
    	reader.read(("data/coffee/"+ss.str()+"_degree.pcd"),*input);



    	// remove NaN values and uninteresting data from Point cloud
    	std::vector<int> indices;
    	pcl::removeNaNFromPointCloud(*input,*transformed_cloud,indices);

    	pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    	pass_x.setInputCloud(input);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(FILTER_MIN,FILTER_MAX);
        pass_x.filter(*transformed_cloud);

    	pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    	pass_y.setInputCloud(transformed_cloud);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(FILTER_MIN,FILTER_MAX);
        pass_y.filter(*transformed_cloud);

    	pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    	pass_z.setInputCloud(transformed_cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(FILTER_MIN,FILTER_MAX);
        pass_z.filter(*transformed_cloud);


        if(first_concat){
    		*concat_cloud = *transformed_cloud;
    	}

    	else{

    		//icp alignment of the transformed cloud
    		//pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
			icp.setInputCloud(transformed_cloud);
        	icp.setInputTarget(concat_cloud);
            icp.setMaxCorrespondenceDistance(MAX_DIST);
            icp.setMaximumIterations(MAX_ITR);
            icp.setTransformationEpsilon(TRANS_EPSILON);
            icp.setRANSACOutlierRejectionThreshold(0.01);


        	//register
        	pcl::PointCloud<pcl::PointXYZRGB> icp_cloud;
        	icp.align(icp_cloud);
        	cerr << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() <<endl;

        	//concatenation of clouds
        	*concat_cloud += icp_cloud; 		//with icp
        	//*concat_cloud += *transformed_cloud;			//without icp alignment
    	}

    	first_concat=false;

//    	//VoxelGrid filter to downsample the cloud
//    	pcl::VoxelGrid<pcl::PointXYZRGB> filter;
//    	filter.setInputCloud (concat_cloud);
//    	filter.setLeafSize (0.01f, 0.01f, 0.01);
//    	filter.filter (*voxel_cloud);
    }

    //Save observed scene to pcd file
    pcl::PCDWriter writer_icp;
    writer_icp.write ("icp.pcd", *concat_cloud, true);


        return 0;

}



