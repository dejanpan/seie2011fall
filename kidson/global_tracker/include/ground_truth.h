/*
 * ground_truth.h
 *
 *  Created on: Aug 18, 2011
 *      Author: engelhar
 */

#ifndef GROUND_TRUTH_H_
#define GROUND_TRUTH_H_

#include "node_definitions.h"

#include <iostream>
#include <fstream>
using namespace std;

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


class stamped_trafo {

public:
 ros::Time time;
 Eigen::Matrix4d trafo;

};


struct Ground_truth {

 Ground_truth(){}
 Ground_truth(string filename);
 vector<stamped_trafo> trafos;

 // assumes monotic increasing stampss
 bool getTrafoAt(ros::Time stamp, stamped_trafo& trafo);

private:
 uint pos;


};









#endif /* GROUND_TRUTH_H_ */
