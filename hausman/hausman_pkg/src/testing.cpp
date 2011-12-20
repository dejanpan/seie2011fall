/*
 * test_app.cpp
 *
 *  Created on: Feb 03, 2011
 *      Author: Christian Bersch
 */

#include <ros/ros.h>

#include <simple_robot_control/robot_control.h>
#include <iostream>
#include <stdio.h>
#include <math.h>

#define PI 3.14159265

void pushing_r(tf::Vector3 point, float step, int iterations,
		simple_robot_control::Robot &robot);

int main(int argc, char** argv) {

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
// main algorithm**************************************************************************************
// first point you wanna reach
	tf::Vector3 point1 = tf::Vector3(0.8, 0.1, 0.0);

//orientation you wanna reach
	tf::Quaternion point_orient = tf::Quaternion(1, 0, 0, 0);

	tf::StampedTransform tf_l1(tf::Transform(point_orient, point1),
			ros::Time::now(), "torso_lift_link", "doesnt_matter");
	robot.left_arm.moveGrippertoPose(tf_l1);

	//put angle in radians around z axis
	float angle1 = PI / 2; //45 degrees
	float z = sin(angle1 / 2);
	float w = cos(angle1 / 2);
	float dist = 0.04;

//		point_orient[2]=z;
//		point_orient[3]=w;
//		point_orient[0]=0;
//		point_orient[1]=0;

	for (int i = 1; i < 7; i++) {
		point1[0] += dist * cos(angle1);
		point1[1] += dist * sin(angle1);

		tf::StampedTransform tf_l2(tf::Transform(point_orient, point1),
				ros::Time::now(), "torso_lift_link", "doesnt_matter");
		if (!robot.left_arm.moveGrippertoPose(tf_l2))

		{
			cout << "An exception occurred at the " << i
					<< ". iteration number. " << endl;
			break;

			sleep(1);
		}
	}
// end of the main algorithm **********************************************************************
// another algorithm- reaching point just towards gripper's x axis

//
//		float step1=0.02;
//			int iterations1=7;
//			int wait_sec1=1;
//			robot.left_arm.moveGrippertoPose(tf_l);
//
//					for (int i=1;i<iterations1;i++)
//					{
////					point1[0]+=i*step1;
//					tf::StampedTransform tf_l3 (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.1+step1,0.0,0.0)), ros::Time::now(), "/l_gripper_l_finger_link","doesnt_matter");
//					robot.left_arm.moveGrippertoPose(tf_l3);
//					sleep(wait_sec1);
//					}

	pushing_r(tf::Vector3(0.6, -0.2, -0.3), 0.03, 7, robot);

	// moving robot with the keyboard
	char direction;

	while (1) {
		cin >> direction;
		switch (direction) {
		case ('w'):
			robot.base.driveForward(0.5);
			break;
		case ('s'):
			robot.base.driveBack(0.5);
			break;
		case ('a'):
			robot.base.driveLeft(0.5);
			break;
		case ('d'):
			robot.base.driveRight(0.5);
			break;
		}

		if (direction == 'q')
			break;
	}

	return 0;

}
// the simplest algorithm with no many directions to choose from

void pushing_r(tf::Vector3 point, float step, int iterations,
		simple_robot_control::Robot &robot) {
	robot.right_arm.moveGripperToPosition(point, "torso_lift_link",
			simple_robot_control::Arm::FRONTAL);

	for (int i = 1; i < iterations; i++) {
		point[0] += i * step;
		robot.right_arm.moveGripperToPosition(point, "torso_lift_link",
				simple_robot_control::Arm::FRONTAL);
		sleep(1);
	}

}

