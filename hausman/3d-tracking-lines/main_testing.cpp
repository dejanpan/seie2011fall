/*
 * region_grow.cpp
 *
 *  Created on: May 30, 2012
 *      Author: vsu
 */

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "PrimitivesExtract.cpp"

unsigned char colormap_ [36] = { 255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0, 255, 0, 255, 0, 255, 255, 127, 0, 0, 0, 127, 0, 0, 0, 127,127, 127, 0, 127, 0, 127, 0, 127, 127 };


int main(int argc, char **argv)
{
  if (argc < 3)
  {
    PCL_INFO ("Usage %s -input_file cloud.pcd \n", argv[0]);

    return -1;
  }

  std::string filename;
  pcl::console::parse_argument(argc, argv, "-input_file", filename);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPCDFile(filename, *cloud);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_boundaries(new pcl::PointCloud<pcl::PointXYZRGBA>);

  std::vector<pcl::ModelCoefficients::Ptr> coefficients;

  pcl::copyPointCloud(*cloud,*cloud_input);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_debug(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_corners(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_lines(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr debug(new pcl::PointCloud<pcl::PointXYZRGBA>);

  std::vector<Eigen::Vector3f> directions_vector;


  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> result_vector;
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> result_vector_lines;

  PrimitivesExtract <pcl::PointXYZRGBA> prim_ex(cloud_input);
//  prim_ex.extractCorners(cloud_input,*result,*result_debug);
  prim_ex.extractCornerVector(cloud_input,result_vector);
  std::cerr<<"number of corners: "<<result_vector.size()<<std::endl;

//  prim_ex.findBoundaries(cloud_input,*cloud_boundaries);
//  prim_ex.extractLines(cloud_boundaries,result_vector_lines,coefficients);
  prim_ex.extractLineVector(cloud_input,result_vector_lines,coefficients,directions_vector);
//  prim_ex.extractCircleVector(cloud_input,result_vector_lines);
  std::cerr<<"number of lines: "<<result_vector_lines.size()<<std::endl;

  pcl::copyPointCloud(*result_vector[0],*result_corners);
  pcl::copyPointCloud(*result_vector_lines[0],*result_lines);




  pcl::visualization::PCLVisualizer viz;
  viz.initCameraParameters();




//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(segments);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> red_color(
						cloud_input, colormap_[0], colormap_[1], colormap_[2]);
	  viz.addPointCloud<pcl::PointXYZRGBA> (cloud_input, red_color);

//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> yellow_color(
//				result_corners, colormap_[9], colormap_[10], colormap_[11]);
//
//		  viz.addPointCloud<pcl::PointXYZRGBA> (result_corners, yellow_color,"result_corners");

//		  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> green_color(
//		  						result, colormap_[3], colormap_[4], colormap_[5]);
//
//		  viz.addPointCloud<pcl::PointXYZRGBA> (result, green_color,"result");

		//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> diff_color(
			//			cloud_boundaries, colormap_[9], colormap_[10], colormap_[11]);
//
//	  viz.addPointCloud<pcl::PointXYZRGBA> (debug, yellow_color,"debug");
//			for (int number=0;number<result_vector.size();number++){
//			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> green_color(
//					result_vector[number], colormap_[3*(number+1)], colormap_[3*(number+1)+1], colormap_[3*(number+1)+2]);
//
//
//
//			std::stringstream k;
//			k<<number+100;
//
//			std::cerr<<"result corners size "<<number<<" = "<<result_vector[number]->size()<<std::endl;
//		  viz.addPointCloud<pcl::PointXYZRGBA> (result_vector[number], green_color,k.str());
//		  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, k.str());
//			}


	for (int number=0;number<result_vector_lines.size();number++){
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> blue_color(
			result_vector_lines[number], colormap_[3*(number+1)], colormap_[3*(number+1)+1], colormap_[3*(number+1)+2]);



	std::stringstream k;
	k<<number;

	std::cerr<<"result lines size "<<number<<" = "<<result_vector_lines[number]->size()<<std::endl;
  viz.addPointCloud<pcl::PointXYZRGBA> (result_vector_lines[number], blue_color,k.str());
  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, k.str());
	}
//  viz.addPointCloud<pcl::PointXYZRGBA> (cloud_boundaries, diff_color,"cloud_boundaries");

//  viz.addPointCloud<pcl::PointXYZRGBA> (result_corners, yellow_color,"result_corners");
	//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> diff_color(
	//			cloud_boundaries, colormap_[9], colormap_[10], colormap_[11]);

  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "result");
  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_boundaries");
  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "result debug");

  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "result_corners");





  viz.spin();

  return 0;
}
