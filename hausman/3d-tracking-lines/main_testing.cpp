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
  pcl::copyPointCloud(*cloud,*cloud_input);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_debug(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_corners(new pcl::PointCloud<pcl::PointXYZRGBA>);


  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> result_vector;

  PrimitivesExtract <pcl::PointXYZRGBA> prim_ex(cloud_input);
//  prim_ex.extractCorners(cloud_input,*result,*result_debug);
  prim_ex.extractCornerVector(cloud_input,result_vector);

  pcl::copyPointCloud(*result_vector[0],*result_corners);



  pcl::visualization::PCLVisualizer viz;
  viz.initCameraParameters();




//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(segments);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> red_color(
						cloud_input, colormap_[0], colormap_[1], colormap_[2]);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> green_color(
						result, colormap_[3], colormap_[4], colormap_[5]);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> blue_color(
							result_debug, colormap_[6], colormap_[7], colormap_[8]);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> yellow_color(
			result_corners, colormap_[9], colormap_[10], colormap_[11]);


  viz.addPointCloud<pcl::PointXYZRGBA> (cloud_input, red_color);
  viz.addPointCloud<pcl::PointXYZRGBA> (result_debug, blue_color,"result_debug");
  viz.addPointCloud<pcl::PointXYZRGBA> (result, green_color,"result");
  viz.addPointCloud<pcl::PointXYZRGBA> (result_corners, yellow_color,"result_corners");


  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "result");
  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "result_debug");
  viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "result_corners");





  viz.spin();

  return 0;
}
