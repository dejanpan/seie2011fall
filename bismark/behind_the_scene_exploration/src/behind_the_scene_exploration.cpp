#include <math.h>
#include "ros/ros.h"
#include "simple_robot_control/robot_control.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl_ros/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "tf/transform_listener.h"
#include "pcl/registration/icp.h"

using namespace std;



int main(int argc, char** argv){


	ros::init(argc, argv, "behind_the_scene_exploration");
	ros::NodeHandle n;

    //Create robot controller interface
    simple_robot_control::Robot robot;


    float r,x,y,z,yaw_angle, pitch_angle,roll ;
    r=0.2;			//radius of the circle trajectory
    z=1.0;			//z offset of the trajectory (wrt. base_link)
    roll = M_PI;
    bool first_concat = true;

    tf::Transform trans;
    tf::Quaternion axis;
    tf::TransformListener tf_;


    sensor_msgs::PointCloud2::ConstPtr input_pc;
    sensor_msgs::PointCloud2::Ptr output_pc_filtered (new sensor_msgs::PointCloud2 ());
    //sensor_msgs::PointCloud2::Ptr concat_pc (new sensor_msgs::PointCloud2 ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_concat_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_transformed_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
//+++++++++++++inital pose++++++++++++++++++++++++++++++++



    robot.torso.move(0.4);
    robot.left_arm.tuck();

    double init_pos_right[]
           = { -1.74, -0.35, -2.36, -2.12, 7.13, -1.00, 3.52,
               -1.74, -0.35, -2.36, -2.12, 7.13, -1.00, 3.52 };
        std::vector<double> init_pos_vec(init_pos_right, init_pos_right+14);
        robot.right_arm.goToJointPos(init_pos_vec);


//+++++++++++++exploring right side of the scene++++++++++++++++++++++++++++++++

    for(int i=220;i<=320;i+=20){

//++++++++++++++++++Pointcloud processing++++++++++++++++++++++++++++++++++++++++

    	//get input pointcloud from rosbag
    	input_pc = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("eye_in_hand/depth_registered/points");
    	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    	pcl::fromROSMsg(*input_pc, *pcl_input_pc);


    	//transformation of Pointcloud
		//bool found_transform = tf_.waitForTransform("eye_in_hand_rgb_optical_frame", "base_link", ros::Time::now(), ros::Duration(10.0));
    	bool found_transform = tf_.waitForTransform( "base_link", "eye_in_hand_rgb_optical_frame", ros::Time::now()-ros::Duration(5), ros::Duration(10.0));


    	if (found_transform)
    	{


    		tf::StampedTransform transform;
    		//tf_.lookupTransform("eye_in_hand_rgb_optical_frame","base_link", ros::Time::now()-ros::Duration(5), transform);
    		pcl_ros::transformPointCloud( "base_link", *pcl_input_pc, *pcl_transformed_pc, tf_);
    		cerr<<"TRANSFORMATION SUCCESS"<<endl;
    	}

    	else
    		continue;

    	if(first_concat){
    		pcl_concat_pc->header = pcl_transformed_pc->header;
    		*pcl_concat_pc = *pcl_transformed_pc;
    	}

    	else
    		*pcl_concat_pc += *pcl_transformed_pc;						//concatenation of clouds

    	cerr<<"concat frame id: "<<pcl_concat_pc->header.frame_id<<endl;
    	cerr<<"transformed frame id: "<<pcl_transformed_pc->header.frame_id<<endl;
    	first_concat=false;


//    	//Filtering of Pointcloud
//    	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
//    	sor.setInputCloud (pcl_concat_pc);
//    	sor.setLeafSize (0.01f, 0.01f, 0.01);
//    	sor.filter (*output_pc_filtered);

//    	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//    	icp.setInputCloud(*pcl_concat_pc);  // boost shared pointer ??!?
//    	icp.setInputTarget(*pcl_transformed_pc); // boost shared pointer ??!?
//    	pcl::PointCloud<pcl::PointXYZRGB> Final;
//    	icp.align(Final);

//++++++++++++++++++END processing Pointcloud++++++++++++++++++++++++++++++++++++++++


    	//x value for circle trajectory
        x= ((cos((M_PI*i)/180))*r)+0.52;
        //y value for circle trajectory
        y= ((sin((M_PI*i)/180))*r)-0.2;

        //angles in radians (gripper looks to the center of the circle)
        yaw_angle = ((float)i/600)*M_PI;
        pitch_angle = 0.1*M_PI;

        //converting angles into quaternions
        axis.setRPY(roll,pitch_angle,yaw_angle);

        //transformation to the new pose (wrt. base_link)
        tf::StampedTransform poses_on_rightside (tf::Transform(axis,tf::Vector3(x,y,z)),
        ros::Time::now(),"base_link","doesnt_matter");
        robot.right_arm.moveGrippertoPose(poses_on_rightside, 10);

        cout<<"degree = "<<i << endl;
        cout<<"pitch = "<<pitch_angle <<"  yaw = "<<yaw_angle<< endl;


    }

    //Save observed scene to pcd file
    pcl::PCDWriter writer;
//    writer.write ("eye_in_hand_scene_downsampled.pcd", *output_pc_filtered,
//        Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

    //Save observed scene without filtering
    writer.write ("eye_in_hand_scene_downsampled.pcd", *pcl_concat_pc);

        return 0;

}



