/*
 * scan.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: vsu
 */

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/random.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

void addNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double noise_std)
{

  boost::mt19937 rng(static_cast<unsigned int> (std::time(0)));
  boost::normal_distribution<float> normal_distrib(0.0f, noise_std * noise_std);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<float> > gaussian_rng(rng, normal_distrib);

  for (size_t cp = 0; cp < cloud->points.size(); cp++)
  {
    cloud->points[cp].z += gaussian_rng();
  }

}

void moveToNewCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double new_center[3])
{
  for (size_t cp = 0; cp < cloud->points.size(); cp++)
  {
    //NOTE: vtk view coordinate system is different than the standard camera coordinates (z forward, y down, x right)
    cloud->points[cp].x += new_center[1];
    cloud->points[cp].y += new_center[2];
    cloud->points[cp].z -= new_center[0];
  }

}

int main(int argc, char** argv)
{

  if (argc < 5)
  {
    PCL_INFO ("Usage %s -input_dir /dir/with/models -output_dir /output/dir [options]\n", argv[0]);
    PCL_INFO (" * where options are:\n"
        "         -height <X>            : simulate scans with sensor mounted on height X\n"
        "         -noise_std <X>         : std of gaussian noise added to pointcloud. Default value 0.0001.\n"
        "         -distance <X>          : simulate scans with object located on a distance X. Can be used multiple times. Default value 4.\n"
        "         -tilt <X>              : tilt sensor for X degrees. X == 0 - sensor looks strait. X < 0 Sensor looks down. X > 0 Sensor looks up . Can be used multiple times. Default value -15.\n"
        "         -shift <X>             : shift object from the straight line. Can be used multiple times. Default value 0.\n"
        "         -num_views <X>         : how many times rotate the object in for every distance, tilt and shift. Default value 6.\n"

        "");
    return -1;
  }

  std::string input_dir, output_dir;
  double height = 1.5;
  double num_views = 6;
  double noise_std = 0.0001;
  std::vector<double> distances;
  std::vector<double> tilt;
  std::vector<double> shift;

  pcl::console::parse_argument(argc, argv, "-input_dir", input_dir);
  pcl::console::parse_argument(argc, argv, "-output_dir", output_dir);
  pcl::console::parse_argument(argc, argv, "-num_views", num_views);
  pcl::console::parse_argument(argc, argv, "-height", height);
  pcl::console::parse_argument(argc, argv, "-noise_std", noise_std);
  pcl::console::parse_multiple_arguments(argc, argv, "-distance", distances);
  pcl::console::parse_multiple_arguments(argc, argv, "-tilt", tilt);
  pcl::console::parse_multiple_arguments(argc, argv, "-shift", shift);

  PCL_INFO("distances size: %d\n", distances.size());
  for (size_t i = 0; i < distances.size(); i++)
  {
    PCL_INFO("distance: %f\n", distances[i]);
  }

  // Set default values if user didn't provide any
  if (distances.size() == 0)
    distances.push_back(4);
  if (tilt.size() == 0)
    tilt.push_back(-15);
  if (shift.size() == 0)
    shift.push_back(0);

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

  // Check if output directory exists
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

  // Iterate over all files
  for (size_t i = 0; i < files_to_process.size(); i++)
  {
    vtkSmartPointer<vtkPolyData> model;
    vtkSmartPointer<vtkPolyDataReader> reader = vtkPolyDataReader::New();
    vtkSmartPointer<vtkTransform> transform = vtkTransform::New();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Compute output directory for this model
    std::vector<std::string> st;
    boost::split(st, files_to_process.at(i), boost::is_any_of("/"), boost::token_compress_on);
    std::string dirname = st.at(st.size() - 1);
    dirname = dirname.substr(0, dirname.size() - 4);
    dirname = output_dir + dirname;

    // Check if output directory for this model exists. If not create
    boost::filesystem::path dirpath(dirname);
    if (!boost::filesystem::exists(dirpath))
    {
      if (!boost::filesystem::create_directories(dirpath))
      {
        PCL_ERROR ("Error creating directory %s.\n", dirpath.c_str ());
        return -1;
      }
    }

    // Load model from file
    reader->SetFileName(files_to_process.at(i).c_str());
    reader->Update();
    model = reader->GetOutput();
    PCL_INFO("Number of points %d\n",model->GetNumberOfPoints());

    // Coumpute bounds and center of the model
    double bounds[6];
    model->GetBounds(bounds);
    double min_z_value = bounds[4];

    double center[3];
    model->GetCenter(center);

    // Initialize PCLVisualizer. Add model to scene.
    pcl::visualization::PCLVisualizer viz;
    viz.initCameraParameters();
    viz.updateCamera();
    viz.setCameraPose(0, 0, 0, 1, 0, 0, 0, 0, 1);
    viz.addModelFromPolyData(model, transform);
    viz.setRepresentationToSurfaceForAllActors();

    // Iterate over all shifts
    for (size_t shift_index = 0; shift_index < shift.size(); shift_index++)
    {

      // Iterate over all tilts
      for (size_t tilt_index = 0; tilt_index < tilt.size(); tilt_index++)
      {

        // Iterate over all distances
        for (size_t distance_index = 0; distance_index < distances.size(); distance_index++)
        {

          // Iterate over all angles
          double angle = 0;
          double angle_step = 360.0 / num_views;
          for (int angle_index = 0; angle_index < num_views; angle_index++)
          {

            // Set transformation with distance, shift, tilt and angle parameters.
            transform->Identity();
            transform->RotateY(tilt[tilt_index]);
            transform->Translate(distances[distance_index], shift[shift_index], -(height + min_z_value));
            transform->RotateZ(angle);

            // Render pointcloud
            viz.renderView(640, 480, cloud);

            //Add noise
            addNoise(cloud, noise_std);

            // Compute new coordinates of the model center
            double new_center[3];
            transform->TransformPoint(center, new_center);

            // Shift origin of the poincloud to the model center
            moveToNewCenter(cloud, new_center);

            // Compute file name for this pointcloud and save it
            std::stringstream ss;
            ss << dirname << "/rotation" << angle << "_distance" << distances[distance_index] << "_tilt"
                << tilt[tilt_index] << "_shift" << shift[shift_index] << ".pcd";
            PCL_INFO("Writing %d points to file %s\n", cloud->points.size(), ss.str().c_str());
            pcl::io::savePCDFile(ss.str(), *cloud);

            // increment angle by step
            angle += angle_step;

          }

        }
      }
    }

  }

  return 0;
}
