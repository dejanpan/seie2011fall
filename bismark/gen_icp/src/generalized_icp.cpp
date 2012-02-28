#include <math.h>
#include "pcl/common/common.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/registration/registration.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/icp_nl.h"
//#include "pcl/registration/gicp.h"
//#include "pcl/registration/bfgs.h"


using namespace std;

//Filter Paramters
float filter_max = 1.5;
float filter_min = 0;
float filter_y_max = 1.3;
float filter_y_min = -1;
//ICP Parameters (lin)
float lin_max_itr = 50;
float lin_max_dist = 0.01;
float lin_trans_epsilon = 1e-9;
float lin_ransac_thres = 0.01;
//ICP Parameters (nonlin)
float nonlin_max_itr = 50;
float nonlin_max_dist = 0.01;
float nonlin_trans_epsilon = 1e-9;
float nonlin_ransac_thres = 0.01;
//ICP Parameters (gen)
float gen_max_dist = 0.01;
float gen_max_itr = 100;
float gen_trans_epsilon = 1e-9;
float gen_ransac_thres = 0.01;
float gen_ransac_itr = 20;
float gen_corr_rand = 20;
float gen_max_opt_itr = 20;
float gen_euc_fit_epsilon = 1e-5;
float gen_rot_epsilon = 0.002;


int main(int argc, char** argv){

    bool first_concat = true;
    bool lin_choice = true;
    bool nonlin_choice = true;
    bool gen_choice = true;
    bool paramter_output = true;



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr concat_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input  (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);



    for(int i=1;i<argc;i+=1){


    	std::stringstream ss;
    	ss<<argv[2];
    	pcl::PCDReader reader;
    	reader.read((ss.str()),*input);



    	// remove NaN values and uninteresting data from cloud
    	pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    	pass_x.setInputCloud(input);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(filter_min,filter_max);
        pass_x.filter(*filtered_cloud);

    	pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    	pass_y.setInputCloud(filtered_cloud);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(filter_y_min,filter_y_max);
        pass_y.filter(*filtered_cloud);

    	pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    	pass_z.setInputCloud(filtered_cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(filter_min,filter_max);
        pass_z.filter(*filtered_cloud);



        //Defining target for upcoming clouds
        if(first_concat){
    		*concat_cloud = *filtered_cloud;
    	}

    	if(!first_concat){

//    		if(strcmp(argv[1],"gen")==0){
//
//    		    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//
//    			//icp alignment of the transformed cloud
//    			icp.setInputCloud(filtered_cloud);
//    			icp.setInputTarget(concat_cloud);
//    			icp.setMaxCorrespondenceDistance(gen_max_dist);									//default = 5
//    			icp.setMaximumIterations(gen_max_itr);											//default = 200
//    			icp.setTransformationEpsilon(gen_trans_epsilon);								//default = 0.0005
//    			//icp.setRANSACOutlierRejectionThreshold(gen_ransac_thres);						//default = 0.05
//    			//icp.setRANSACIterations(gen_ransac_itr);										//default = 1.45e+09
//    			icp.setCorrespondenceRandomness(gen_corr_rand);									//default = 20
//    			icp.setMaximumOptimizerIterations(gen_max_opt_itr);								//default = 20
//    			icp.setEuclideanFitnessEpsilon(gen_euc_fit_epsilon);							//default = -1.79769e+308
//    			//icp.setRotationEpsilon(gen_rot_epsilon);										//default = 0.002
//
//				if(paramter_output){
//                cout <<endl<< "Using generalized ICP with following parameters:"<< endl << endl;
//            	cerr << "max correspondence distance:" << icp.getMaxCorrespondenceDistance() <<endl;
//            	cerr << "max iterations:" << icp.getMaximumIterations()<<endl;
//            	cerr << "RANSAC Iterations:" << icp.getRANSACIterations() <<endl;
//            	cerr << "RANSAC outlier rejection threshold:" << icp.getRANSACOutlierRejectionThreshold()<<endl;
//            	cerr << "transformation epsilon:" << icp.getTransformationEpsilon() <<endl;
//            	cerr << "correspondence randomness:" << icp.getCorrespondenceRandomness() <<endl;
//            	cerr << "max optimizer iterations:" << icp.getMaximumOptimizerIterations() <<endl;
//            	cerr << "euclidean fitness epsilon:" << icp.getEuclideanFitnessEpsilon() <<endl;
//            	cerr << "rotation epsilon:" << icp.getRotationEpsilon() <<endl<<endl;
//				paramter_output = false;
//				}
//
//              //register
//              icp.align(*icp_cloud);
//            	//concatenation of clouds
//            	*concat_cloud += *icp_cloud;
//
//            	cerr << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() <<endl;
//            	cerr << "Transformation:" << endl << endl<< icp.getFinalTransformation() <<endl<<endl;
//    		}
//
//    		else {
//    			gen_choice = false;
//    		}


    		if(strcmp(argv[1],"lin")==0){

    			pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    			//icp alignment of the transformed cloud
    			icp.setInputCloud(filtered_cloud);
    			icp.setInputTarget(concat_cloud);
                icp.setMaxCorrespondenceDistance(lin_max_dist);								//default = 1.34078e+154
                icp.setMaximumIterations(lin_max_itr);										//default = 0
                icp.setTransformationEpsilon(lin_trans_epsilon);							//default = 0
                icp.setRANSACOutlierRejectionThreshold(lin_ransac_thres);					//default = 0.05

            	if(paramter_output){
                cout <<endl<< "Using linear ICP with following parameters:"<< endl<< endl;
            	cerr << "max correspondence distance:" << icp.getMaxCorrespondenceDistance() <<endl;
            	cerr << "max iterations:" << icp.getMaximumIterations()<<endl;
            	cerr << "transformation epsilon:" << icp.getTransformationEpsilon() <<endl;
            	cerr << "RANSAC outlier rejection threshold:" << icp.getRANSACOutlierRejectionThreshold()<<endl<<endl;
				paramter_output = false;
				}


                //register
                icp.align(*icp_cloud);
            	//concatenation of clouds
            	*concat_cloud += *icp_cloud;

            	cerr << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() <<endl;
            	cerr << "Transformation:" << endl << endl<< icp.getFinalTransformation() <<endl<<endl;
    		}

    		else {
    			lin_choice = false;
    		}


    		if(strcmp(argv[1],"nonlin")==0){

    		    pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    			//icp alignment of the transformed cloud
    			icp.setInputCloud(filtered_cloud);
    			icp.setInputTarget(concat_cloud);
                icp.setMaxCorrespondenceDistance(nonlin_max_dist);								//default = 1.34078e+154
                icp.setMaximumIterations(nonlin_max_itr);										//default = 32619
                icp.setTransformationEpsilon(nonlin_trans_epsilon);								//default = 0
                icp.setRANSACOutlierRejectionThreshold(nonlin_ransac_thres);					//default = 0.05

            	if(paramter_output){
                cout <<endl<< "Using nonlinear ICP with following parameters:"<< endl<< endl;
            	cerr << "max correspondence distance:" << icp.getMaxCorrespondenceDistance() <<endl;
            	cerr << "max iterations:" << icp.getMaximumIterations()<<endl;
            	cerr << "transformation epsilon:" << icp.getTransformationEpsilon() <<endl;
            	cerr << "RANSAC outlier rejection threshold:" << icp.getRANSACOutlierRejectionThreshold()<<endl<<endl;
				paramter_output = false;
				}

                //register
                icp.align(*icp_cloud);
            	//concatenation of clouds
            	*concat_cloud += *icp_cloud;

            	cerr << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() <<endl;
            	cerr << "Transformation:" << endl << endl<< icp.getFinalTransformation() <<endl<<endl;
    		}

    		else {
    			nonlin_choice = false;
    		}
    	}

    	first_concat = false;
    }

	if(!lin_choice && !nonlin_choice && !gen_choice){
		cout <<endl<<"not the correct argument, use:"<<endl<<endl<<"lin for linear icp"<<endl<<"nonlin for nonlinear icp"<<endl<<"gen for generalized icp"<<endl;
	return 0;
	}





    //Save observed scene to pcd file
 	std::stringstream icp_choice;
	icp_choice<<argv[1];
    pcl::PCDWriter writer_icp;
    writer_icp.write (icp_choice.str()+"_icp.pcd", *concat_cloud, true);

    return 0;


}



