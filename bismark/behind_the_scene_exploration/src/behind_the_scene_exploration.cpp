#include <ros/ros.h>
#include <simple_robot_control/robot_control.h>
#include <math.h>

using namespace std;

int main(int argc, char** argv){

	ros::init(argc, argv, "robot_control_test_app");
    ros::NodeHandle nh;

    //Create robot controller interface
    simple_robot_control::Robot robot;




    float r,x,y,z,yaw_angle, pitch_angle;
    //radius of the circle trajectory
    r=0.2;

    //z offset of the trajectory (wrt. base_link)
    z=0.75;

    tf::Transform trans;
    tf::Quaternion axis;



//+++++++++++++exploring right side of the scene++++++++++++++++++++++++++++++++

    for(int i=220;i<=320;i+=20){

            //x value for circle trajectory
            x= ((cos((M_PI*i)/180))*r)+0.52;
            //y value for circle trajectory
            y= ((sin((M_PI*i)/180))*r)-0.2;

            //angles in radians (gripper looks to the center of the circle)
            yaw_angle = ((float)i/640)*M_PI;
            pitch_angle = 0.1*M_PI;

            //converting angles into quaternions
            axis.setRPY(0.0,pitch_angle,yaw_angle);

            //transformation to the new pose (wrt. base_link)
            tf::StampedTransform poses_on_rightside (tf::Transform(axis,tf::Vector3(x,y,z)),
            ros::Time::now(),"base_link","doesnt_matter");
            robot.right_arm.moveGrippertoPose(poses_on_rightside, 10);

            cout<<"degree = "<<i << endl;
            cout<<"pitch = "<<pitch_angle <<"  yaw = "<<yaw_angle<< endl;


    }

//+++++++++++++top view of the scene++++++++++++++++++++++++++++++++

        yaw_angle = 0;
        pitch_angle = 0.3*M_PI;
        axis.setRPY(0.0,pitch_angle,yaw_angle);


        tf::StampedTransform topview_pose (tf::Transform(axis,
        	tf::Vector3(0.9,-0.1,0.9)), ros::Time::now(),"base_link","doesnt_matter");
        robot.right_arm.moveGrippertoPose(topview_pose, 20);



//+++++++++++++exploring left side of the scene++++++++++++++++++++++++++++++++

    for(int i=60;i<=140;i+=20){

            //x value for circle trajectory
            x= ((cos((M_PI*i)/180))*r)+0.5;
            //y value for circle trajectory
            y= ((sin((M_PI*i)/180))*r)-0.25;

            //angles in radians (gripper looks to the center of the circle)
            yaw_angle = 1.5*M_PI;
            pitch_angle = 0.1*M_PI;

            //converting angles into quaternions
            axis.setRPY(0.0,pitch_angle,yaw_angle);

            //transformation to the new pose (wrt. base_link)
            tf::StampedTransform poses_on_leftside (tf::Transform(axis,tf::Vector3(x,y,z)),
            ros::Time::now(),"base_link","doesnt_matter");
            robot.right_arm.moveGrippertoPose(poses_on_leftside, 20);

            cout<<"degree = "<<i << endl;
            cout<<"pitch = "<<pitch_angle <<"  yaw = "<<yaw_angle<< endl;


    }






        return 0;

}



