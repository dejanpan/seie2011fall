#include <iostream>
#include <pcl/io/pcd_io.h>
#include <svm.h>

int
main (int argc, char *argv[])
{
  // Parse Arguments
  // TODO
  std::string incloudfile = argv[1];
  std::string features_file_name = argv[2];
  std::string labels_file_name = argv[3];

  const int feature_length = 24;

  // Load the query feature
  pcl::PointCloud<pcl::Histogram<feature_length>>::Ptr cloud (new pcl::PointCloud<pcl::Histogram<feature_length>>);
  pcl::io::loadPCDFile (incloudfile.c_str (), *cloud);

  //////////////////////////////////////////////////////////////////////////////
  // Support vector machine classification
  struct svm_parameter param;
  struct svm_problem prob;
  struct svm_model *model;

  // set parameter
  param.svm_type = C_SVC;
  param.kernel_type = RBF;
  param.degree = 3;
  param.gamma = 0;
  param.coef0 = 0;
  param.nu = 0.5;
  param.cache_size = 100;
  param.C = 1;
  param.eps = 1e-3;
  param.p = 0.1;
  param.shrinking = 1;
  param.probability = 0;
  param.nr_weight = 0;
  param.weight_label = NULL;
  param.weight = NULL;

  // set problem
  prob.l =


  model = svm_train(&prob, &param);


  return (0);
}
