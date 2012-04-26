#include <iostream>
#include <pcl/io/pcd_io.h>
#include </home/stefan/ros_workspace/perception_pcl_electric_unstable/pcl/build/pcl_trunk/apps/include/pcl/apps/nn_classification.h>

int
main (int argc, char *argv[])
{
  // Parse Arguments
  // TODO
  std::string incloudfile = argv[1];
  std::string features_file_name = argv[2];
  std::string labels_file_name = argv[3];

  // Load the query feature
//  pcl::PointCloud<pcl::Histogram<feature_length>>::Ptr cloud (new pcl::PointCloud<pcl::Histogram<feature_length>>);
//  pcl::io::loadPCDFile (incloudfile.c_str (), *cloud);

  // Read the first line of a file
  std::string line;
  std::ifstream reader (incloudfile, std::ios::in);
  if (reader.is_open ())
  {
    getline (reader, line);
    reader.close ();
  }

  // parse the line
  std::vector<std::string> v;
  split (v, line, " ");
  pcl::PointCloud<pcl::Histogram<v.size()-1> >::Ptr cloud (new pcl::PointCloud<pcl::Histogram<v.size()-1>>);

  //std::cout << "label: " << v[0] << std::endl;
  for (size_t i = 1; i < v.size(); ++i)
  {
    std::vector<std::string> w;
    split (w, v[i], ":");
    //std::cout << i << ":" << w[1] << std::endl;
    cloud->points[0].histogram[i-1] = atof (w[1].c_str ());
  }

  // Nearest neighbors classification
  pcl::NNClassification<pcl::Histogram<v.size()-1> > nn;
  nn.loadTrainingFeatures (features_file_name, labels_file_name);
  pcl::NNClassification<pcl::Histogram<v.size()-1> >::ResultPtr result = nn.classify(cloud->points[0], 300, 50);


  return (0);
}
