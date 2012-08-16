/*
 * TrajectoryClustering.cpp
 *
 *  Created on: Aug 15, 2012
 *      Author: Karol Hausman
 */

#include "textureless_objects_tracking/TrajectoryClustering.h"



TrajectoryClustering::TrajectoryClustering() {
	// TODO Auto-generated constructor stub

}

TrajectoryClustering::~TrajectoryClustering() {
	// TODO Auto-generated destructor stub
}

double TrajectoryClustering::calcDistance(Eigen::MatrixXf source,Eigen::MatrixXf target){

	double xd=source(0,0)-target(0,0);
	double yd=source(1,0)-target(1,0);
	double zd=source(2,0)-target(2,0);

	double dist=sqrt(xd*xd+yd*yd+zd*zd);

	return dist;
}

void TrajectoryClustering::buildLaplacian(double threshold,double percentage){

	   Eigen::MatrixXf breaks=Eigen::MatrixXf::Zero(features_number_,features_number_);
	   Eigen::MatrixXf laplacian=Eigen::MatrixXf::Zero(features_number_,features_number_);

	  //resize all the columns to the same size
	  std::vector<size_t> sizes;
	  for(int i=0;i<grouped_features_.size();i++)
		  sizes.push_back(grouped_features_[i].size());
	  size_t min_size=sizes[0];
	  for(int i=0;i<sizes.size();i++)	{
		  if(sizes[i]<min_size)
			  min_size=sizes[i];
	  }

	  std::cerr<<"min size: "<<min_size<<std::endl;

	  for(int i=0;i<grouped_features_.size();i++){

		  std::cerr<<"sizes before: "<<grouped_features_[i].size()<<std::endl;
		  grouped_features_[i].resize(min_size);
		  std::cerr<<"sizes after: "<<grouped_features_[i].size()<<std::endl;
	  }



	  double distance=0;
	  double reference_distance=0;
	  for(int i=0;i<features_number_;i++){
		  for(int j=i+1;j<features_number_;j++){
			  for(int k=0;k<min_size;k++){
				  distance = calcDistance(grouped_features_[i][k],grouped_features_[j][k]);
				  reference_distance=calcDistance(grouped_features_[i][0],grouped_features_[j][0]);
				  if(distance-reference_distance>threshold){
					  breaks(i,j)=breaks(i,j)+1;
					  breaks(j,i)=breaks(j,i)+1;
				  }
			  }
		  }
	  }

	  //normalizing
	  breaks=breaks/min_size;
	  std::cerr<<"breaks matrix: "<<std::endl;
	  std::cerr<<breaks<<std::endl;


	  for(int i=0;i<features_number_;i++){
		  for(int j=i+1;j<features_number_;j++){

			  if(breaks(i,j)<percentage){
				  laplacian(i,j)=1;
				  laplacian(j,i)=1;
			  }

		  }
	  }
	  Eigen::VectorXf degrees;
	  Eigen::MatrixXf degrees_m;

	  degrees=(laplacian.colwise().sum()).cast<float>();
	  degrees_m=degrees.asDiagonal();

	  laplacian=degrees_m-laplacian;
	  std::cerr<<"laplacian matrix: "<<std::endl;
	  std::cerr<<laplacian<<std::endl;

	  laplacian.eigenvalues();

	  std::cerr<<"eigen_values matrix: "<<std::endl;
	  std::cerr<<laplacian.eigenvalues()<<std::endl;

}
void TrajectoryClustering::readFromFile(std::string fname,int features_number){

      std::ifstream infile;

      const char * fname_char = fname.c_str();


      infile.open (fname_char, std::ifstream::in);
	  double d=0;
	  int counter_till_six=0;
	  int counter_till_three_lines=0;

//	  std::vector <std::vector<Eigen::MatrixXf> > grouped_features;
	  features_number_=features_number;
	  size_t size=features_number;
	  grouped_features_.resize(features_number);

	  Eigen::MatrixXf pose(3,2);

	  std::vector< Eigen::MatrixXf> ungrouped_features;

      while (infile.good()){

    	  infile >> d;
//    	  Eigen::Matrix3d rotation;
//    	  btMatrix3x3 rotationMatrix;
//    	  rotationMatrix.setEulerYPR(0, 0, 0);
//    	  btVector3 position;
    	  if(counter_till_six==6){
    		  counter_till_six=0;
    		  ungrouped_features.push_back(pose);
    	  }

    	  if(counter_till_six==0)
    		  pose(0,0)=d;
    	  else if (counter_till_six==1)
    		  pose(1,0)=d;
    	  else if (counter_till_six==2)
    		  pose(2,0)=d;
    	  else if (counter_till_six==3)
    		  pose(0,1)=d;
    	  else if (counter_till_six==4)
    		  pose(1,1)=d;
    	  else if (counter_till_six==5){
    		  pose(2,1)=d;

    	  }

    	  counter_till_six++;

      }


      for(int i=0;i<ungrouped_features.size();i++)
    	  grouped_features_[i%features_number].push_back(ungrouped_features[i]);


//		  std::cout<<"feature column 1: "<<grouped_features[0][1]<<std::endl;

      infile.close();
}
