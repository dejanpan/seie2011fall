/*
 * main_testing.cpp
 *
 *  Created on: May 30, 2012
 *      Author: Karol Hausman
 */

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "PrimitivesExtract.cpp"
#include <ros/ros.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>

#include <object_part_decomposition/point_type.h>
#include "textureless_objects_tracking/point_type.h"
#include "textureless_objects_tracking/PushPointEstimation.h"
#include "textureless_objects_tracking/TrajectoryClustering.h"
#include "textureless_objects_tracking/DenseReconstruction.h"



unsigned char colormap_ [36] = { 255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0, 255, 0, 255, 0, 255, 255, 127, 0, 0, 0, 127, 0, 0, 0, 127,127, 127, 0, 127, 0, 127, 0, 127, 127 };

void planeSegmentation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
		pcl::ModelCoefficients &coefficients, pcl::PointIndices &inliers) {
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud);
	seg.segment(inliers, coefficients);
}
void removeZeroPoints(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA> &result) {
	for (size_t i = 0; i < cloud->points.size(); i++) {
		pcl::PointXYZRGBA point = cloud->points[i];
		if (!(fabs(point.x) < 0.01 && fabs(point.y) < 0.01
				&& fabs(point.z) < 0.01) && !isnan(point.x)
				&& !isnan(point.y) && !isnan(point.z))
			result.points.push_back(point);
	}

	result.width = result.points.size();
	result.height = 1;
	result.is_dense = true;
}
int main(int argc, char **argv)
{

	ros::init(argc, argv, "image_tracker");

  if (argc < 3)
  {
    PCL_INFO ("Usage %s -input_file cloud.pcd \n", argv[0]);

    return -1;
  }

  std::string filename;
  pcl::console::parse_argument(argc, argv, "-input_file", filename);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPCDFile(filename, *cloud);

  pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_for_dense(new pcl::PointCloud<pcl::PointXYZLRegionF>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_for_dense_viz(new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::io::loadPCDFile("afterwards_apart.pcd", *cloud_for_dense);

  DenseReconstruction dense(cloud_for_dense);
  std::cout<<"size of operational: "<<dense.cloud_operational_->size()<<std::endl;

  pcl::copyPointCloud(*(dense.cloud_operational_),*cloud_for_dense_viz);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segments(new pcl::PointCloud<pcl::PointXYZRGB>);

  dense.regionGrowing(segments);

  std::cerr<<"segments size: "<<segments->size()<<std::endl;


  std::vector<pcl::PointIndices::Ptr> ind_vec;

  dense.extractEuclideanClustersCurvature(ind_vec);

  dense.mergeClusters(ind_vec);

      pcl::io::savePCDFile("after_merging.pcd",*dense.cloud_operational_);

      dense.addSideWall(ind_vec);

  std::cerr<<"CLUSTERS size: "<<ind_vec.size()<<std::endl;

  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters_vec;

//  for (int i=0;i<ind_vec.size();i++){
//
//	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGBA>);
//
//
//	  pcl::copyPointCloud(*(dense.region_grow_point_cloud_),*ind_vec[i],*cloud_temp);
////	  clusters_vec.push_back(cloud_temp);
//  }

  for (int i=0;i<dense.clusters_vec_only_boudaries.size();i++){

	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGBA>);

	  pcl::copyPointCloud(*(dense.clusters_vec_only_boudaries[i]),*cloud_temp);

	  clusters_vec.push_back(cloud_temp);


  }



  std::cerr<<"CLUSTERS PCL size: "<<clusters_vec.size()<<std::endl;



  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_boundaries(new pcl::PointCloud<pcl::PointXYZRGBA>);

  std::vector<pcl::ModelCoefficients::Ptr> coefficients;

  pcl::copyPointCloud(*cloud,*cloud_input);

  std::cout<<"height of the cloud"<<cloud_input->height<<std::endl;
    std::cout<<"width of the cloud"<<cloud_input->width<<std::endl;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_debug(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_corners(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_lines(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr debug(new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr grasps_points(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_virtual(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_objects_in_virt_cam(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_objects_in_virt_cam_no_nans(new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_concave(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_convex(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr push_point_cloud_result(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr push_point_result(new pcl::PointCloud<pcl::PointXYZRGBA>);



  pcl::PointCloud<pcl::PointXYZLRegion>::Ptr cloud_concave_push(new pcl::PointCloud<pcl::PointXYZLRegion>);
  pcl::PointCloud<pcl::PointXYZLRegion>::Ptr cloud_convex_push(new pcl::PointCloud<pcl::PointXYZLRegion>);

  pcl::PointCloud<pcl::PointXYZLRegion>::Ptr push_point_cloud(new pcl::PointCloud<pcl::PointXYZLRegion>);
  pcl::PointCloud<pcl::PointXYZLRegion>::Ptr cloud_objects_in_virt_cam_regions(new pcl::PointCloud<pcl::PointXYZLRegion>);
  pcl::PointCloud<pcl::PointXYZLRegion>::Ptr push_point_xlregion(new pcl::PointCloud<pcl::PointXYZLRegion>);


  std::vector<Eigen::Vector3f> directions_vector;


  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> result_vector;
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> result_vector_lines;

  PrimitivesExtract <pcl::PointXYZRGBA> prim_ex(cloud_input);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients);

	std::cout<<"size of the cloud_input cloud: "<<cloud_input->size()<<std::endl;


	planeSegmentation(cloud_input, *coefficients2,
			*inliers);
	prim_ex.setPlaneCoefficients(coefficients2);

	 pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	  	  extract.setInputCloud (cloud_input);
	  	  extract.setIndices (inliers);
	  	  extract.setNegative (true);
	  	  extract.filter (*cloud_objects_in_virt_cam);
//  prim_ex.extractCorners(cloud_input,*result,*result_debug);
  prim_ex.extractCornerVector(cloud_objects_in_virt_cam,result_vector,1);

  prim_ex.getAll3DCornersFromService(cloud_objects_in_virt_cam,*cloud_concave,*cloud_convex);
  std::cerr<<"number of corners: "<<result_vector.size()<<std::endl;

//  pcl::copyPointCloud(prim_ex.getConvex_corners(),*grasps_points);
  pcl::copyPointCloud(*cloud_concave,*grasps_points);
  pcl::copyPointCloud(*cloud_convex,*debug);


  pcl::copyPointCloud(*cloud_concave,*cloud_concave_push);
  pcl::copyPointCloud(*cloud_convex,*cloud_convex_push);


//  pcl::copyPointCloud(*cloud_objects_in_virt_cam,*cloud_objects_in_virt_cam_regions);

//	pcl::PointCloud<pcl::PointXYZLRegion>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZLRegion>);
//  pcl::io::savePCDFile("objects_in_the_virtual_cam.pcd",*cloud_objects_in_virt_cam);
  removeZeroPoints(cloud_objects_in_virt_cam,*cloud_objects_in_virt_cam_no_nans);
	prim_ex.getSegments(cloud_objects_in_virt_cam_no_nans,cloud_objects_in_virt_cam_regions);
//  pcl::io::savePCDFile("objects_in_the_virtual_cam_regions.pcd",*cloud_objects_in_virt_cam_regions);

  PushPointEstimation push_point_es;
  push_point_es.setAll3DCornersFromService(*cloud_concave_push,*cloud_convex_push);
//  push_point_es.getLabels(cloud_objects_in_virt_cam_regions);
	std::cout<<"size of the cloud_objects_in_virt_cam cloud: "<<cloud_objects_in_virt_cam->size()<<std::endl;

	std::cout<<"size of the cloud_objects_in_virt_cam_regions cloud: "<<cloud_objects_in_virt_cam_regions->size()<<std::endl;

	pcl::PointCloud<pcl::PointXYZLRegion> push_point;
  push_point_es.getPushCloud(cloud_objects_in_virt_cam_regions,*push_point_cloud,push_point);
    pcl::copyPointCloud(*push_point_cloud,*push_point_cloud_result);

//    push_point_xlregion->points.push_back(push_point);
    pcl::copyPointCloud(push_point,*push_point_result);



    TrajectoryClustering clustering;
    clustering.readFromFile("poses_final.txt",3);
    clustering.buildLaplacian();



//  prim_ex.findBoundaries(cloud_input,*cloud_boundaries);
//  prim_ex.extractLines(cloud_boundaries,result_vector_lines,coefficients);
//  prim_ex.extractLineVector(cloud_input,result_vector_lines,directions_vector);
//  prim_ex.extractCircleVector(cloud_input,result_vector_lines);
//  std::cerr<<"number of lines: "<<result_vector_lines.size()<<std::endl;

//  pcl::copyPointCloud(*result_vector[0],*result_corners);
//  pcl::copyPointCloud(*result_vector_lines[0],*result_lines);

//Convert point cloud to an image and depthmap of type OpenCV Mat
  /*
    cv::Mat cvimage(cloud_input->width,cloud_input->height, CV_8UC3);
        cv::Mat cvimage_depth(cloud->width,cloud->height, CV_32FC1);
    //cv::Mat cvimage_xyz(cloud->width,cloud->height, CV_32FC3);
  //  cv::Mat depth_mask;// = cv::Mat::ones(cloud->width,cloud->height, CV_MAT_DEPTH_MASK );
        for (uint h = 0; h < cloud_input->height; h++) {
                for (uint w = 0; w < cloud_input->width; w++) {
            //Get colour data for our cvimage
                        cvimage.at<cv::Vec3b>(w, h)[0] = cloud_input->at(h * cloud_input->width + w).b;
                        cvimage.at<cv::Vec3b>(w, h)[1] = cloud_input->at(h * cloud_input->width + w).g;
                        cvimage.at<cv::Vec3b>(w, h)[2] = cloud_input->at(h * cloud_input->width + w).r;
            //Get depth data
                        cvimage_depth.at<float>(w, h) = cloud_input->at(h * cloud_input->width + w).z;
           // cvimage_xyz.at<cv::Vec3f>(w, h)[0] = cloud->at(h * cloud->width + w).x;
           // cvimage_xyz.at<cv::Vec3f>(w, h)[1] = cloud->at(h * cloud->width + w).y;
           // cvimage_xyz.at<cv::Vec3f>(w, h)[2] = cloud->at(h * cloud->width + w).z;
                }
        }
    //Transpose
        cvimage = cvimage.t();
        cvimage_depth = cvimage_depth.t();
        textureless_objects_tracking::cornerFind::Response res_corner;
        cv::Mat bw_image(cvimage.rows,cvimage.cols,CV_8U);
/*
		cv::cvtColor(cvimage,bw_image ,CV_BGR2GRAY);

        prim_ex.getCornersToPush(bw_image,res_corner);
        std::cout<<"res_corner: "<<res_corner.corner.size()<<std::endl;
        std::cout<<"res_corner convex: "<<res_corner.corner_convex.size()<<std::endl;
//
        prim_ex.get3dPoints(res_corner,*grasps_points);
        std::cout<<"grasps points: "<<grasps_points->points.size()<<std::endl;
*/
/* 	  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  	  extract.setInputCloud (cloud_input);
  	  extract.setIndices (inliers);
  	  extract.setNegative (true);
  	  extract.filter (*cloud_objects_in_virt_cam);





        cv::Mat top_image(cvimage.rows,cvimage.cols,CV_8U);
        prim_ex.getTopView(cloud_objects_in_virt_cam,top_image);
        cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
           cv::imshow( "Display window", top_image );

            cv::waitKey(0);

		prim_ex.getCornersToPush(top_image,res_corner);
		std::cout<<"res_corner: "<<res_corner.corner.size()<<std::endl;
		std::cout<<"res_corner convex: "<<res_corner.corner_convex.size()<<std::endl;

		prim_ex.get3dPoints(res_corner);
		std::cout<<"grasps points: "<<grasps_points->points.size()<<std::endl;
*/
//    cv::namedWindow( "Display window image", CV_WINDOW_AUTOSIZE );// Create a window for display.
//    cv::imshow( "Display window image", cvimage );                   // Show our image inside it.
//
//    cv::namedWindow( "Display window depth", CV_WINDOW_AUTOSIZE );// Create a window for display.
//    cv::imshow( "Display window depth", cvimage_depth );                   // Show our image inside it.
//
//
//    cv::waitKey(0);                                          // Wait for a keystroke in the window


//	    pcl::io::savePCDFile("test.pcd",*cloud_input);
//	    pcl::io::savePCDFile("test2.pcd",*cloud_virtual);

  pcl::visualization::PCLVisualizer viz;
  viz.initCameraParameters();




//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(segments);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> red_color(
//			cloud_input, colormap_[0], colormap_[1], colormap_[2]);
//	  viz.addPointCloud<pcl::PointXYZRGBA> (cloud_for_dense_viz, red_color);

//		  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_color(
//		  			segments, colormap_[3], colormap_[4], colormap_[5]);
//
//		  viz.addPointCloud<pcl::PointXYZRGB> (segments, green_color,"result");

//		  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(segments);
//		  viz.addPointCloud<pcl::PointXYZRGB> (segments, rgb,"result");


		  			for (int number=0;number<clusters_vec.size();number++){
		  			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> green_color(
		  					clusters_vec[number], colormap_[3*(number+1)], colormap_[3*(number+1)+1], colormap_[3*(number+1)+2]);



		  			std::stringstream k;
		  			k<<number+100;

//		  			std::cerr<<"result corners size "<<number<<" = "<<result_vector[number]->size()<<std::endl;
		  		  viz.addPointCloud<pcl::PointXYZRGBA> (clusters_vec[number], green_color,k.str());
//		  		  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, k.str());
		  			}

/*

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> yellow_color(
				push_point_result, colormap_[9], colormap_[10], colormap_[11]);

		  viz.addPointCloud<pcl::PointXYZRGBA> (push_point_result, yellow_color,"debug");
		  /*
		//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> diff_color(
			//			cloud_boundaries, colormap_[9], colormap_[10], colormap_[11]);
//
//	  viz.addPointCloud<pcl::PointXYZRGBA> (debug, yellow_color,"debug");
/*			for (int number=0;number<result_vector.size();number++){
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> green_color(
					result_vector[number], colormap_[3*(number+1)], colormap_[3*(number+1)+1], colormap_[3*(number+1)+2]);



			std::stringstream k;
			k<<number+100;

			std::cerr<<"result corners size "<<number<<" = "<<result_vector[number]->size()<<std::endl;
		  viz.addPointCloud<pcl::PointXYZRGBA> (result_vector[number], green_color,k.str());
		  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, k.str());
			}
*/

//	for (int number=0;number<result_vector_lines.size();number++){
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> blue_color(
//			result_vector_lines[number], colormap_[3*(number+1)], colormap_[3*(number+1)+1], colormap_[3*(number+1)+2]);
//
//
//
//	std::stringstream k;
//	k<<number;

//	std::cerr<<"result lines size "<<number<<" = "<<result_vector_lines[number]->size()<<std::endl;
//  viz.addPointCloud<pcl::PointXYZRGBA> (result_vector_lines[number], blue_color,k.str());
//  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, k.str());
//	}
//  viz.addPointCloud<pcl::PointXYZRGBA> (cloud_boundaries, diff_color,"cloud_boundaries");

//  viz.addPointCloud<pcl::PointXYZRGBA> (result_corners, yellow_color,"result_corners");
	//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> diff_color(
	//			cloud_boundaries, colormap_[9], colormap_[10], colormap_[11]);

//  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "result");
  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_boundaries");
  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "result debug");
  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "debug");

  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "result_corners");





  viz.spin();



  return 0;
}
