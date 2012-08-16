/*
 * TrajectoryClustering.h
 *
 *  Created on: Aug 15, 2012
 *      Author: Karol Hausman
 */

#ifndef TRAJECTORYCLUSTERING_H_
#define TRAJECTORYCLUSTERING_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <math.h>
#include <Eigen/Eigenvalues>

class TrajectoryClustering {
public:
	TrajectoryClustering();
	virtual ~TrajectoryClustering();
	void readFromFile(std::string fname,int features_number);
	double calcDistance(Eigen::MatrixXf source,Eigen::MatrixXf target);
	void buildLaplacian(double threshold=0.014,double percentage=0.03);


	std::vector <std::vector<Eigen::MatrixXf> > grouped_features_;
	int features_number_;

};

#endif /* TRAJECTORYCLUSTERING_H_ */
