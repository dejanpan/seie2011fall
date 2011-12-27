#include <ros/ros.h>
#include <simple_robot_control/robot_control.h>
#include <math.h>

using namespace std;

int main(int argc, char** argv){

	ros::init(argc, argv, "robot_control_test_app");
    ros::NodeHandle nh;

    //Create robot controller interface
    simple_robot_control::Robot robot;


    //specify grab pose with postion and orientation as StampedTransform
    tf::StampedTransform tf_init (tf::Transform(tf::Quaternion(0,0,0,1),
    	tf::Vector3(0.8,0.0,1.1)),
        ros::Time::now(),"base_link","doesnt_matter");
    robot.right_arm.moveGrippertoPose(tf_init);

//+++++++++++++++++++++++++++++++++TEST++++++++++++++++++++++++++++++++++++++++
    for(int i=0;i<=360;i+=20){
			float r,x,y;

            // radius of the circle trajectory
            r=0.1;
            //x value for circle trajectory
            x= ((cos((M_PI*i)/180))*r)+0.46;
            //y value for circle trajectory
            y= ((sin((M_PI*i)/180))*r)-0.15;

            tf::Transform trans;
            tf::Quaternion axis;

            //angles in radians (gripper looks to the center of the circle)
			float yaw_angle = (M_PI+(M_PI*((float)i/180)));
            float pitch_angle = 0.24*M_PI;

            //converting angles into quaternions
            axis.setRPY(0.0,pitch_angle,yaw_angle);

            //transformation to the new pose (wrt. base_link)
            tf::StampedTransform update (tf::Transform(axis,tf::Vector3(x,y,0.5)),
            ros::Time::now(),"base_link","doesnt_matter");
            cout<<"degree = "<<i << endl;
            cout<<"pitch = "<<pitch_angle <<"  yaw = "<<yaw_angle<< endl;
            robot.right_arm.moveGrippertoPose(update, 10);

    }

        return 0;

}



