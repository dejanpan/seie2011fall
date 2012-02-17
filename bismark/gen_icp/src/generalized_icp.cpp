#include <math.h>
#include "pcl-1.5/pcl/pcl_base.h"
#include "pcl-1.5/pcl/point_types.h"
#include "pcl-1.5/pcl/io/pcd_io.h"
#include "pcl-1.5/pcl/filters/voxel_grid.h"
#include "pcl-1.5/pcl/filters/passthrough.h"
#include "pcl-1.5/pcl/registration/registration.h"
#include "pcl-1.5/pcl/registration/icp.h"
#include "pcl-1.5/pcl/registration/icp_nl.h"
#include "pcl-1.5/pcl/registration/gicp.h"
#include "pcl-1.5/pcl/registration/bfgs.h"

using namespace std;

//Filter Paramters
#define FILTER_MAX 1.5
#define FILTER_MIN 0
#define FILTER_Y_MAX 1.3
#define FILTER_Y_MIN -1
//ICP Parameters
#define MAX_ITR 50
#define MAX_DIST 0.01
#define TRANS_EPSILON 1e-4
#define RANSAC_THRESHOLD 0.01



int main(int argc, char** argv){

    bool first_concat = true;



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr concat_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input  (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nan_input  (new pcl::PointCloud<pcl::PointXYZRGB>);



    //for(int i=160;i<=280;i+=20){
    for(int i=220;i<=320;i+=20){

    	std::stringstream ss;
    	ss<<i;
    	pcl::PCDReader reader;
    	reader.read(("data/coffee/"+ss.str()+"_degree.pcd"),*nan_input);



    	// remove NaN values and uninteresting data from Point cloud
    	std::vector<int> indices;
    	pcl::removeNaNFromPointCloud(*nan_input,*input,indices);



    	pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    	pass_x.setInputCloud(input);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(FILTER_MIN,FILTER_MAX);
        pass_x.filter(*transformed_cloud);

    	pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    	pass_y.setInputCloud(transformed_cloud);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-1,1.3);
        pass_y.filter(*transformed_cloud);

    	pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    	pass_z.setInputCloud(transformed_cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(FILTER_MIN,FILTER_MAX);
        pass_z.filter(*transformed_cloud);




        if(first_concat){
    		*concat_cloud = *transformed_cloud;
    	}

    	else{

    		if(strcmp(argv[1],"gen")==0){

    		    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    			//icp alignment of the transformed cloud
    			icp.setInputCloud(transformed_cloud);
    			icp.setInputTarget(concat_cloud);
    			icp.setMaxCorrespondenceDistance(0.01);									//default = 5
    			icp.setMaximumIterations(100);											//default = 200
    			icp.setTransformationEpsilon(0.005);									//default = 0.0005
    			//icp.setRANSACOutlierRejectionThreshold(0.01);							//default = 0.05
    			//icp.setRANSACIterations();											//default = 1.45e+09
    			icp.setCorrespondenceRandomness(20);									//default = 20
    			icp.setMaximumOptimizerIterations(20);									//default = 20
    			icp.setEuclideanFitnessEpsilon(1e-5);									//default = -1.79769e+308
    			//icp.setRotationEpsilon(2.0);											//default = 0.002

                cout <<endl<< "Using generalized ICP with following parameters:"<< endl << endl;
            	cerr << "max correspondence distance:" << icp.getMaxCorrespondenceDistance() <<endl;
            	cerr << "max iterations:" << icp.getMaximumIterations()<<endl;
            	cerr << "RANSAC Iterations:" << icp.getRANSACIterations() <<endl;
            	cerr << "RANSAC outlier rejection threshold:" << icp.getRANSACOutlierRejectionThreshold()<<endl;
            	cerr << "transformation epsilon:" << icp.getTransformationEpsilon() <<endl;
            	cerr << "correspondence randomness:" << icp.getCorrespondenceRandomness() <<endl;
            	cerr << "max optimizer iterations:" << icp.getMaximumOptimizerIterations() <<endl;
            	cerr << "euclidean fitness epsilon:" << icp.getEuclideanFitnessEpsilon() <<endl;
            	cerr << "rotation epsilon:" << icp.getRotationEpsilon() <<endl<<endl;

            	//register
            	pcl::PointCloud<pcl::PointXYZRGB> icp_cloud;
            	icp.align(icp_cloud);
            	cerr << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() <<endl;

            	//concatenation of clouds
            	*concat_cloud += icp_cloud;

    		}


    		if(strcmp(argv[1],"lin")==0){

    			pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    			//icp alignment of the transformed cloud
    			icp.setInputCloud(transformed_cloud);
    			icp.setInputTarget(concat_cloud);
                icp.setMaxCorrespondenceDistance(MAX_DIST);								//default = 1.34078e+154
                icp.setMaximumIterations(MAX_ITR);										//default = 0
                icp.setTransformationEpsilon(TRANS_EPSILON);							//default = 0
                icp.setRANSACOutlierRejectionThreshold(RANSAC_THRESHOLD);				//default = 0.05

                cout <<endl<< "Using linear ICP with following parameters:"<< endl<< endl;
            	cerr << "max correspondence distance:" << icp.getMaxCorrespondenceDistance() <<endl;
            	cerr << "max iterations:" << icp.getMaximumIterations()<<endl;
            	cerr << "transformation epsilon:" << icp.getTransformationEpsilon() <<endl;
            	cerr << "RANSAC outlier rejection threshold:" << icp.getRANSACOutlierRejectionThreshold()<<endl<<endl;

            	//register
            	pcl::PointCloud<pcl::PointXYZRGB> icp_cloud;
            	icp.align(icp_cloud);
            	cerr << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() <<endl;

            	//concatenation of clouds
            	*concat_cloud += icp_cloud;

    		}


    		if(strcmp(argv[1],"nonlin")==0){

    		    pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    			//icp alignment of the transformed cloud
    			icp.setInputCloud(transformed_cloud);
    			icp.setInputTarget(concat_cloud);
                icp.setMaxCorrespondenceDistance(MAX_DIST);								//default = 1.34078e+154
                icp.setMaximumIterations(MAX_ITR);										//default = 32619
                icp.setTransformationEpsilon(TRANS_EPSILON);							//default = 0
                icp.setRANSACOutlierRejectionThreshold(RANSAC_THRESHOLD);				//default = 0.05

                cout <<endl<< "Using nonlinear ICP with following parameters:"<< endl<< endl;
            	cerr << "max correspondence distance:" << icp.getMaxCorrespondenceDistance() <<endl;
            	cerr << "max iterations:" << icp.getMaximumIterations()<<endl;
            	cerr << "transformation epsilon:" << icp.getTransformationEpsilon() <<endl;
            	cerr << "RANSAC outlier rejection threshold:" << icp.getRANSACOutlierRejectionThreshold()<<endl<<endl;

            	//register
            	pcl::PointCloud<pcl::PointXYZRGB> icp_cloud;
            	icp.align(icp_cloud);
            	cerr << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() <<endl;

            	//concatenation of clouds
            	*concat_cloud += icp_cloud;

    		}

    		else{
    			cout << "not the correct argument"<<endl;
    		}

    	}

    	first_concat=false;

    }




    //Save observed scene to pcd file
 	std::stringstream icp_choice;
	icp_choice<<argv[1];
    pcl::PCDWriter writer_icp;
    writer_icp.write (icp_choice.str()+"_icp.pcd", *concat_cloud, true);

        return 0;


}



