/*
 * classify.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: vsu
 */

#include <training.h>

int main(int argc, char** argv)
{
  std::string database_file_name = "data/codebook.yaml";

  std::map<featureType, std::map<std::string, std::vector<Eigen::Vector4f> > > codebook;
  load_codebook(database_file_name, codebook);
  save_codebook("data/cb.yaml", codebook);



  return 0;
}
