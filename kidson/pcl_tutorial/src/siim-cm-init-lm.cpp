/*
 * siim-cm-init-lm.cpp
 *
 *  Created on: Mar 13, 2012
 *      Author: darko490
 */

#include "siimcloudmatch.h"
#include <pcl/registration/transformation_estimation_lm.h>

void
SIIMCloudMatch::getInitialTransformRGBDLM(void)
{
    TransformationEstimationWDF<pcl::PointXYZRGB,pcl::PointXYZRGB> initialTransformWDF;

    float alpha = 0.3;
    initialTransformWDF.setAlpha(alpha);
    initialTransformWDF.setCorrespondecesDFP(*SurfCorrespondingPairs_denan);

    std::vector<int> vec_1, vec_2;
    initialTransformWDF.estimateRigidTransformation(*(frameTarget->cloud_denan), vec_1, *(frameStart->cloud_denan), vec_2,
    							transformSurfRGBDLM);
}

void
SIIMCloudMatch::getTransformRGBDICP(Eigen::Matrix4f& init_tr)
{

	boost::shared_ptr< TransformationEstimationWDF<pcl::PointXYZRGB,pcl::PointXYZRGB> >
		initialTransformWDF(new TransformationEstimationWDF<pcl::PointXYZRGB,pcl::PointXYZRGB>());

    float alpha = 0.3;
    initialTransformWDF->setAlpha(alpha);
    initialTransformWDF->setCorrespondecesDFP(*SurfCorrespondingPairs_denan);

    // Instantiate ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (75);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);

    // Set TransformationEstimationWDF as ICP transform estimator
    icp.setTransformationEstimation (initialTransformWDF);

    icp.setInputCloud( frameTarget->cloud_denan );
    icp.setInputTarget( frameStart->cloud_denan );

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed( new pcl::PointCloud<pcl::PointXYZRGB>);
    // As before, due to my initial bad naming, it is the "target" that is being transformed
    //									set initial transform
    icp.align ( *cloud_transformed, init_tr );
    std::cout << "[SIIMCloudMatch::runICPMatch] Has converged? = " << icp.hasConverged() << std::endl <<
    				"	fitness score (SSD): " << icp.getFitnessScore (1000) << std::endl;
    transformSurfRGBDICP = icp.getFinalTransformation ();

    /// Final ICP transformation is obtained by multiplying initial transformed with icp refinement
}

void
SIIMCloudMatch::getInitialTransformLM(void)
{
    pcl::registration::TransformationEstimationLM<pcl::PointXYZRGB,pcl::PointXYZRGB> initialTransformLM;
	/** Since there are problems if there is initial transform is not given, one will be built from first 4 points
	 *	in pcl::Correspondences
	 */
//    pcl::WarpPointRigid6D<pcl::PointXYZRGB, pcl::PointXYZRGB> warp_f;

//    initialTransformLM.setWarpFunction(warp_f);

    initialTransformLM.estimateRigidTransformation(*(frameTarget->cloud_denan), *(frameStart->cloud_denan),
    							*SurfCorrespondingPairs_denan, transformSurfLM);
}

void
SIIMCloudMatch::runICPMatch(Eigen::Matrix4f init_tr)
{
    if (rgbTransformDone)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (0.05);
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (75);
        // Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon (1e-8);
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon (1);

        // Change transformation estimation to from SVD to LM:	DID NOT HELP
//        boost::shared_ptr< pcl::registration::TransformationEstimationLM<pcl::PointXYZRGB,pcl::PointXYZRGB> >
//        	init_tr_estimator_LM(new pcl::registration::TransformationEstimationLM<pcl::PointXYZRGB,pcl::PointXYZRGB>());
//        icp.setTransformationEstimation (init_tr_estimator_LM);

        // Actually, initial transform can be set in align method: icp.align(cloud_tr, init_tr)
        /// First transform second cloud with initial transform
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed( new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud( *(frameTarget->cloud_denan), *cloud_transformed, init_tr);

        icp.setInputCloud( frameTarget->cloud_denan );
        icp.setInputTarget( cloud_transformed );

        // As before, due to my initial bad naming, it is the "target" that is being transformed
        icp.align ( *cloud_transformed );
        std::cout << "[SIIMCloudMatch::runICPMatch] Has converged? = " << icp.hasConverged() << std::endl <<
        				"	fitness score (SSD): " << icp.getFitnessScore (1000) << std::endl;
        tICP = icp.getFinalTransformation ();

        /// Final ICP transformation is obtained by multiplying initial transformed with icp refinement
        tICP = tICP * init_tr;
    }
}
