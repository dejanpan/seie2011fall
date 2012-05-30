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

int main(int argc, char **argv)
{
  std::string filename = "data/scans/Chairs/Aluminium_Group_EA_119_0000F152-centered/full.pcd";
  float move_x = 10.0;
  float move_y = 20.0;
  float rotate_z = M_PI/3;

  pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>), model_transformed(new pcl::PointCloud<
      pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointNormal>::Ptr model_normals(new pcl::PointCloud<pcl::PointNormal>),
                                         model_transformed_normals(new pcl::PointCloud<pcl::PointNormal>);

  pcl::io::loadPCDFile(filename, *model);

  Eigen::Affine3f true_transform;
  true_transform.setIdentity();
  true_transform.translate(Eigen::Vector3f(move_x, move_y, 0));
  true_transform.rotate(Eigen::AngleAxisf(rotate_z, Eigen::Vector3f(0, 0, 1)));

  std::cerr << "Transforming points with model (" << move_x << "," << move_y << "," << rotate_z << ")" << std::endl;
  pcl::transformPointCloud(*model, *model_transformed, true_transform);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals(true);

  // Set parameters
  mls.setPolynomialFit(true);
  mls.setPolynomialOrder(2);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);

  // Reconstruct
  mls.setInputCloud(model);
  mls.process(*model_normals);

  mls.setInputCloud(model_transformed);
  mls.process(*model_transformed_normals);

  pcl::SampleConsensusModel3DOF<pcl::PointNormal>::Ptr
                                                       model_3dof(
                                                                  new pcl::SampleConsensusModel3DOF<pcl::PointNormal>(
                                                                                                                      model_normals));
  model_3dof->setTarget(model_transformed_normals);


  pcl::RandomSampleConsensus<pcl::PointNormal> ransac(model_3dof);
  ransac.setDistanceThreshold(.001);
  ransac.setProbability(0.99);
  ransac.computeModel();

  Eigen::VectorXf coeff(3);
  ransac.getModelCoefficients(coeff);

  std::cerr << "Ransac model (" << coeff[0] << "," << coeff[1] << "," << coeff[2] << ")" << std::endl;

  return 0;
}
