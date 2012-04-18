/*
 * scan.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: vsu
 */

#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{

  std::string input_dir, output_dir;
  double height = 2.0;
  double num_views = 6;
  std::vector<double> distances;

  pcl::console::parse_argument(argc, argv, "-input_dir", input_dir);
  pcl::console::parse_argument(argc, argv, "-output_dir", output_dir);
  pcl::console::parse_argument(argc, argv, "-num_views", num_views);
  pcl::console::parse_argument(argc, argv, "-height", height);
  pcl::console::parse_multiple_arguments(argc, argv, "-distance", distances);

  PCL_INFO("distances size: %d\n", distances.size());
  for (int i = 0; i < distances.size(); i++)
  {
    PCL_INFO("distance: %f\n", distances[i]);
  }

  if (distances.size() == 0)
    distances.push_back(3);

  // Check if input path exists
  boost::filesystem::path input_path(input_dir);
  if (!boost::filesystem::exists(input_path))
  {
    PCL_ERROR("Input directory doesnt exists.");
    return -1;
  }

  // Check if input path is a directory
  if (!boost::filesystem::is_directory(input_path))
  {
    PCL_ERROR("%s is not directory.", input_path.c_str());
    return -1;
  }

  boost::filesystem::path output_path(output_dir);
  if (!boost::filesystem::exists(output_path) || !boost::filesystem::is_directory(output_path))
  {
    if (!boost::filesystem::create_directories(output_path))
    {
      PCL_ERROR ("Error creating directory %s.\n", output_path.c_str ());
      return -1;
    }
  }

  // Find all .vtk files in the input directory
  std::vector<std::string> files_to_process;
  PCL_INFO("Processing following files:\n");
  boost::filesystem::directory_iterator end_iter;
  for (boost::filesystem::directory_iterator iter(input_path); iter != end_iter; iter++)
  {
    boost::filesystem::path file(*iter);
    if (file.extension() == ".vtk")
    {
      files_to_process.push_back(file.c_str());
      PCL_INFO("\t%s\n", file.c_str());
    }

  }

  // Check if there are any .vtk files to process
  if (files_to_process.size() == 0)
  {
    PCL_ERROR("Directory %s has no .vtk files.", input_path.c_str());
    return -1;
  }

  for (size_t i = 0; i < files_to_process.size(); i++)
  {
    vtkSmartPointer<vtkPolyData> model;
    vtkSmartPointer<vtkPolyDataReader> reader = vtkPolyDataReader::New();
    vtkSmartPointer<vtkTransform> transform = vtkTransform::New();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    std::vector<std::string> st;
    boost::split(st, files_to_process.at(i), boost::is_any_of("/"), boost::token_compress_on);
    std::string dirname = st.at(st.size() - 1);
    dirname = dirname.substr(0, dirname.size() - 4);
    dirname = output_dir + dirname;

    boost::filesystem::path dirpath(dirname);
    if (!boost::filesystem::exists(dirpath))
    {
      if (!boost::filesystem::create_directories(dirpath))
      {
        PCL_ERROR ("Error creating directory %s.\n", dirpath.c_str ());
        return -1;
      }
    }

    reader->SetFileName(files_to_process.at(i).c_str());
    reader->Update();
    model = reader->GetOutput();
    PCL_INFO("Number of points %d\n",model->GetNumberOfPoints());

    double bounds[6];
    model->GetBounds(bounds);
    double min_z_value = bounds[4];

    //pcl::visualization::PCLVisualizer viz;
    //viz.initCameraParameters();
    //viz.addModelFromPolyData(model, transform);
    //viz.setRepresentationToSurfaceForAllActors();

    for (size_t i = 0; i < distances.size(); i++)
    {
      pcl::visualization::PCLVisualizer viz;
      viz.initCameraParameters();
      viz.updateCamera();
      viz.addModelFromPolyData(model, transform);
      viz.setRepresentationToSurfaceForAllActors();
      viz.setCameraPosition(distances[i], 0, height + min_z_value, 0, 0, (bounds[4] + bounds[5]) / 2);
      //viz.updateCamera();


      transform->Identity();

      double angle = 360.0 / num_views;
      for (int j = 0; j < num_views; j++)
      {
        viz.renderView(640, 480, cloud);
        transform->RotateZ(angle);

        std::stringstream ss;
        ss << dirname << "/" << j << "_" << i << ".pcd";
        pcl::io::savePCDFile(ss.str(), *cloud);

      }

      viz.close();

    }

  }

  return 0;
}
