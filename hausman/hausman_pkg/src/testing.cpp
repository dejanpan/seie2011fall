/*
 * test_app.cpp
 *
 *  Created on: Feb 03, 2011
 *      Author: Christian Bersch
 */


#include <ros/ros.h>

#include <simple_robot_control/robot_control.h>
#include <iostream>

void pushing_r(tf::Vector3 point,float step,int iterations,simple_robot_control::Robot &robot );


int main(int argc, char** argv){

	using namespace std;


	ros::init(argc, argv, "robot_control_test_app");
	ros::NodeHandle nh;

	//Create robot controller interface
	simple_robot_control::Robot robot;

	//look straight
	robot.head.lookat("torso_lift_link", tf::Vector3(0.1, 0.0, 0.0));

	//do stuff with arms
	robot.left_arm.tuck();
	robot.right_arm.stretch();

//	double tuck_pos_right[] = { -0.4,0.0,0.0,-2.25,0.0,0.0,0.0, -0.01,1.35,-1.92,-1.68, 1.35,-0.18,0.31};
//	std::vector<double> tuck_pos_vec(tuck_pos_right, tuck_pos_right+14);
//	robot.right_arm.goToJointPos(tuck_pos_vec);
//
//	robot.right_arm.stretch();
//
	tf::Vector3 point1=tf::Vector3(0.8,0.1,0.0);

	tf::StampedTransform tf_l (tf::Transform(tf::Quaternion(0,0,0.1,1), point1), ros::Time::now(), "torso_lift_link","doesnt_matter");
//		robot.left_arm.moveGrippertoPose(tf_l);

	tf::Vector3 tmp=tf::Vector3(0.9,0.1,0.0);
	tmp= tf_l*point1;
//
//		float step1=0.02;
//			int iterations1=7;
//			int wait_sec1=1;
//			robot.left_arm.moveGrippertoPose(tf_l);
//
//					for (int i=1;i<iterations1;i++)
//					{
////					point1[0]+=i*step1;
//					tf::StampedTransform tf_l1 (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.1+step1,0.0,0.0)), ros::Time::now(), "/l_gripper_l_finger_link","doesnt_matter");
//					robot.left_arm.moveGrippertoPose(tf_l1);
//					sleep(wait_sec1);
//					}
//




//	pushing_r(tf::Vector3(0.6,-0.2, -0.3),0.03 ,7 ,robot);

	char direction;

	while(1){
		cin >> direction;
		switch (direction)
		{
		case ('w') :robot.base.driveForward(0.5);
			break;
		case ('s') :robot.base.driveBack(0.5);
					break;
		case ('a') :robot.base.driveLeft(0.5);
					break;
		case ('d') :robot.base.driveRight(0.5);
					break;
		}

		if (direction=='q')break;
	}




	tf::Vector3 point=tf::Vector3(0.6,-0.2, -0.3);
	float step=0.03;
	int iterations=7;
	int wait_sec=1;
	robot.right_arm.moveGripperToPosition(point, "torso_lift_link", simple_robot_control::Arm::FRONTAL);

			for (int i=1;i<iterations;i++)
			{
			point[0]+=i*step;
			robot.right_arm.moveGripperToPosition(point, "torso_lift_link", simple_robot_control::Arm::FRONTAL);
			sleep(wait_sec);
			}






//	robot.right_arm.moveGripperToPosition(tf::Vector3(0.6,0.0, 0.0), "torso_lift_link", simple_robot_control::Arm::FROM_ABOVE);
//	robot.right_arm.moveGripperToPosition(tf::Vector3(0.6,0.0, 0.0), "torso_lift_link", simple_robot_control::Arm::FRONTAL);
//	float step=0.03;
//	for (int i=1;i<11;i++)
//	{
//	float move=0.6+i*step;
//	robot.right_arm.moveGripperToPosition(tf::Vector3(move,0.0, 0.0), "torso_lift_link", simple_robot_control::Arm::FRONTAL);
//	sleep(1);
//	}
//	robot.right_arm.moveGripperToPosition(tf::Vector3(1.2, 0.0, 0.0), "torso_lift_link", simple_robot_control::Arm::FRONTAL);

//	tf::StampedTransform tf_l (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.8,0.1,0.0)), ros::Time::now(), "torso_lift_link","doesnt_matter");
//	robot.left_arm.moveGrippertoPose(tf_l);
//
//	//look at left gripper
//	robot.head.lookat("l_gripper_tool_frame");
//
//	//drive 0.5m forward
//	robot.base.driveForward(0.5);
//
//	//raise torso to 10cm above lowest position
//	robot.torso.move(0.1);

	return 0;

}

void pushing_r(tf::Vector3 point,float step,int iterations,simple_robot_control::Robot &robot )
{
	robot.right_arm.moveGripperToPosition(point, "torso_lift_link", simple_robot_control::Arm::FRONTAL);

		for (int i=1;i<iterations;i++)
		{
		point[0]+=i*step;
		robot.right_arm.moveGripperToPosition(point, "torso_lift_link", simple_robot_control::Arm::FRONTAL);
		sleep(1);
		}

}


