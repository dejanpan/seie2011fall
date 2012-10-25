/*
 * import_mesh.cpp
 *
 *  Imports a mesh file and converts to pcd
 *
 *  Created on: 25/10/2012
 *      Author: ross
 */

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <ros/console.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Pointcloud;

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    ROS_INFO("please provide a filename ");
    exit(0);
  }

  Pointcloud::Ptr output (new Pointcloud);

  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile(argv[1],mesh);
  pcl::fromROSMsg(mesh.cloud, *output);

  //pcl::PLYReader reader;
  //reader.read (argv[1], *output);

  pcl::PCDWriter writer;
  writer.write ("pcd_file.pcd", *output,false);

  return 0;
}
