/*
 * sac_test.cpp
 *
 *  Created on: May 30, 2012
 *      Author: vsu
 */

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <sac_3dof.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
  std::string filename = "data/scans/Chairs/Aluminium_Group_EA_119_0000F152-centered/full.pcd";
  float move_x = 10.0;
  float move_y = 20.0;
  float rotate_z = M_PI / 3;

  pcl::PointCloud<pcl::PointNormal>::Ptr model(new pcl::PointCloud<pcl::PointNormal>),
                                         model_transformed(new pcl::PointCloud<pcl::PointNormal>);

  pcl::io::loadPCDFile(filename, *model);

  Eigen::Affine3f true_transform;
  true_transform.setIdentity();
  true_transform.translate(Eigen::Vector3f(move_x, move_y, 0));
  true_transform.rotate(Eigen::AngleAxisf(rotate_z, Eigen::Vector3f(0, 0, 1)));

  std::cerr << "Transforming points with model (" << move_x << "," << move_y << "," << rotate_z << ")" << std::endl;
  pcl::transformPointCloudWithNormals(*model, *model_transformed, true_transform);

//  pcl::visualization::PCLVisualizer viz;
//  viz.initCameraParameters();
//
//  viz.addPointCloudNormals<pcl::PointNormal> (model, 1, 0.01, "cloud");
//  viz.addPointCloudNormals<pcl::PointNormal> (model_transformed, 1, 0.01, "cloud1");
//  viz.spin();

  pcl::SampleConsensusModel3DOF<pcl::PointNormal>::Ptr
                                                       model_3dof(
                                                                  new pcl::SampleConsensusModel3DOF<pcl::PointNormal>(
                                                                                                                      model));
  model_3dof->setTarget(model_transformed);

  pcl::RandomSampleConsensus<pcl::PointNormal> ransac(model_3dof);
  ransac.setDistanceThreshold(.001);
  ransac.setProbability(0.99999);
  ransac.computeModel();

  Eigen::VectorXf coeff(3);
  ransac.getModelCoefficients(coeff);

  std::cerr << "Ransac model (" << coeff[0] << "," << coeff[1] << "," << coeff[2] << ")" << std::endl;

  return 0;
}
