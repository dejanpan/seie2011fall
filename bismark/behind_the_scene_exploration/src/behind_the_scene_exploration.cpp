#include <math.h>
#include "ros/ros.h"
#include "simple_robot_control/robot_control.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl_ros/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "tf/transform_listener.h"

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

    tf::Transform trans;
    tf::Quaternion axis;
    tf::TransformListener tf_;

//    sensor_msgs::PointCloud2 input_pc, transformed_pc;

    sensor_msgs::PointCloud2::Ptr input_pc (new sensor_msgs::PointCloud2 ());
    sensor_msgs::PointCloud2::Ptr transformed_pc (new sensor_msgs::PointCloud2 ());
    sensor_msgs::PointCloud2::Ptr output_pc (new sensor_msgs::PointCloud2 ());


//+++++++++++++inital pose++++++++++++++++++++++++++++++++



    robot.torso.move(0.4);

    double init_pos_right[]
           = { -1.74, -0.35, -2.36, -2.12, 7.13, -1.00, 3.52,
               -1.74, -0.35, -2.36, -2.12, 7.13, -1.00, 3.52 };
        std::vector<double> init_pos_vec(init_pos_right, init_pos_right+14);
        robot.right_arm.goToJointPos(init_pos_vec);





//+++++++++++++exploring right side of the scene++++++++++++++++++++++++++++++++

    for(int i=220;i<=320;i+=20){

//++++++++++++++++++Pointcloud processing++++++++++++++++++++++++++++++++++++++++




		  //transformation of Pointcloud
		  bool found_transform = tf_.waitForTransform("eye_in_hand_rgb_optical_frame", "base_link",
													ros::Time::now(), ros::Duration(10.0));

    	  if (found_transform)
    	  {
    	    tf::StampedTransform transform;
    	    tf_.lookupTransform("base_link","eye_in_hand_rgb_optical_frame", ros::Time::now(), transform);
    	    pcl_ros::transformPointCloud("base_link", transform, *input_pc, *transformed_pc);
    	  }


    	  //Convert PC2 to PointXYZ and concatenate the Pointclouds
    	  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_transformed_pc (new pcl::PointCloud<pcl::PointXYZ>);
    	  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_concat_pc (new pcl::PointCloud<pcl::PointXYZ>);
    	  pcl::fromROSMsg(*transformed_pc, *pcl_transformed_pc);
    	  *pcl_concat_pc += *pcl_transformed_pc;

    	  //Filtering of Pointcloud
    	  sensor_msgs::PointCloud2::Ptr output_pc_filtered (new sensor_msgs::PointCloud2 ());
    	  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    	  sor.setInputCloud (pcl_concat_pc);
    	  sor.setLeafSize (0.01f, 0.01f, 0.01f);
    	  sor.filter (*output_pc_filtered);

    	  //Save to pcd file
    	  pcl::PCDWriter writer;
    	  writer.write ("eye_in_hand_scene_downsampled.pcd", *output_pc_filtered,
    	         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);



//++++++++++++++++++END processing Pointcloud++++++++++++++++++++++++++++++++++++++++


            //x value for circle trajectory
            x= ((cos((M_PI*i)/180))*r)+0.52;
            //y value for circle trajectory
            y= ((sin((M_PI*i)/180))*r)-0.2;

            //angles in radians (gripper looks to the center of the circle)
            yaw_angle = ((float)i/640)*M_PI;
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



        return 0;

}



