#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/sgf1.h>
#include <pcl/features/sgf2.h>
#include <pcl/features/sgf3.h>
#include <pcl/features/sgf4.h>
#include <pcl/features/sgf5.h>
#include <pcl/features/sgf6.h>
#include <pcl/features/sgf7.h>
#include <pcl/features/sgf8.h>
#include <pcl/features/sgf9.h>


typedef unsigned int SET;

#define IS_ELEMENT_OF( var, set ) ( set == (set | (SET) (1 << (var-1))) )

void split (std::vector<std::string> & theStringVector, const std::string  & theString, const std::string  & theDelimiter);
int bin2dec(const char *bin);
char* strrev( char* s );


int
main (int argc, char *argv[])
{
  size_t i;
  size_t feature_counter = 0;

  const int sgf_size = 24;
  pcl::PointCloud<pcl::Histogram<sgf_size> >::Ptr sgfs (new pcl::PointCloud<pcl::Histogram<sgf_size> > ());
  sgfs->width = 1;
  sgfs->height = 1;
  sgfs->points.resize (1);
  std::string indices_filename;

  // default values
  SET features = 1023;
  int label = 0;
  bool is_set_ind_file = false;

  // parse options
  for (i = 1; i < argc; ++i)
  {
    if (argv[i][0] != '-')
    {
      break;
    }
    if(++i >= argc)
    {
      return (-1); //exit_with_help();
    }
    switch (argv[i-1][1])
    {
      case 'f':
        features = bin2dec (strrev(argv[i]));
        break;
      case 'l':
        label = atoi (argv[i]);
        break;
      case 'i':
        indices_filename = argv[i];
        is_set_ind_file = true;
        break;
      default:
        fprintf(stderr,"Unknown option: -%c\n", argv[i-1][1]);
        return (-1); //exit_with_help();
    }
  }

  // determine filenames
  if (i >= argc-1)
  {
    return (-1); //exit_with_help();
  }
  std::string incloudfile = argv[i];
  std::string feature_filename = argv[++i];


  // Load cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (incloudfile.c_str (), *cloud);


  // Create a set of indices
  std::vector<int> indices (cloud->points.size ());
  if (is_set_ind_file)
  {
    // TDOD: read indices file and copy values into indices vector
  }
  else
  {
    for (size_t i = 0; i < indices.size (); ++i)
    {
      indices[i] = i;
    }
  }
  boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));


  /////////////////////////////////////////////////////////////////////////////////////
  // Feature 1
  if (IS_ELEMENT_OF(1, features))
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::Histogram<pcl::SGF1_SIZE> >::Ptr sgf1s (new pcl::PointCloud<pcl::Histogram<pcl::SGF1_SIZE> > ());
    pcl::SGF1Estimation<pcl::PointXYZ, pcl::Histogram<pcl::SGF1_SIZE> > sgf1;
    sgf1.setInputCloud (cloud);
    sgf1.setIndices(indicesptr);
    sgf1.setSearchMethod (tree);
    sgf1.setKSearch (10);
    sgf1.compute (*sgf1s);

    for (size_t n = 0; n < pcl::SGF1_SIZE; ++n)
    {
      sgfs->points[0].histogram[feature_counter + n] = sgf1s->points[0].histogram[n];
    }
    feature_counter += pcl::SGF1_SIZE;

    // Print the result
    std::cout << "Feature SGF1: " << sgf1s->points[0].histogram[0] << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////////////////
  // Feature 2
  if (IS_ELEMENT_OF(2, features))
  {
    pcl::PointCloud<pcl::Histogram<pcl::SGF2_SIZE> >::Ptr sgf2s (new pcl::PointCloud<pcl::Histogram<pcl::SGF2_SIZE> > ());
    pcl::SGF2Estimation<pcl::PointXYZ, pcl::Histogram<pcl::SGF2_SIZE> > sgf2;
    sgf2.setInputCloud (cloud);
    sgf2.setIndices(indicesptr);
    sgf2.compute (*sgf2s);

    for (size_t n = 0; n < pcl::SGF2_SIZE; ++n)
    {
      sgfs->points[0].histogram[feature_counter + n] = sgf2s->points[0].histogram[n];
    }
    feature_counter += pcl::SGF2_SIZE;

    // Print the result
    std::cout << "Feature SGF2: " << sgf2s->points[0].histogram[0] << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////////////////
  // Feature 3
  if (IS_ELEMENT_OF(3, features))
  {
    pcl::PointCloud<pcl::Histogram<pcl::SGF3_SIZE> >::Ptr sgf3s (new pcl::PointCloud<pcl::Histogram<pcl::SGF3_SIZE> > ());
    pcl::SGF3Estimation<pcl::PointXYZ, pcl::Histogram<pcl::SGF3_SIZE> > sgf3;
    sgf3.setInputCloud (cloud);
    sgf3.setIndices(indicesptr);
    sgf3.compute (*sgf3s);

    for (size_t n = 0; n < pcl::SGF3_SIZE; ++n)
    {
      sgfs->points[0].histogram[feature_counter + n] = sgf3s->points[0].histogram[n];
    }
    feature_counter += pcl::SGF3_SIZE;

    // Print the result
    std::cout << "Feature SGF3: " << sgf3s->points[0].histogram[0] << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////////////////
  // Feature 4
  if (IS_ELEMENT_OF(4, features))
  {
    pcl::PointCloud<pcl::Histogram<pcl::SGF4_SIZE> >::Ptr sgf4s (new pcl::PointCloud<pcl::Histogram<pcl::SGF4_SIZE> > ());
    pcl::SGF4Estimation<pcl::PointXYZ, pcl::Histogram<pcl::SGF4_SIZE> > sgf4;
    sgf4.setInputCloud (cloud);
    sgf4.setIndices(indicesptr);
    sgf4.compute (*sgf4s);

    for (size_t n = 0; n < pcl::SGF4_SIZE; ++n)
    {
      sgfs->points[0].histogram[feature_counter + n] = sgf4s->points[0].histogram[n];
    }
    feature_counter += pcl::SGF4_SIZE;

    // Print the result
    std::cout << "Feature SGF4: (";
    for (size_t idx = 0; idx < pcl::SGF4_SIZE - 1; ++idx)
    {
      std::cout << sgf4s->points[0].histogram[idx] << ", ";
    }
    std::cout << sgf4s->points[0].histogram[pcl::SGF4_SIZE - 1] << ")" << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////////////////
  // Feature 5
  if (IS_ELEMENT_OF(5, features))
  {
    pcl::PointCloud<pcl::Histogram<pcl::SGF5_SIZE> >::Ptr sgf5s (new pcl::PointCloud<pcl::Histogram<pcl::SGF5_SIZE> > ());
    pcl::SGF5Estimation<pcl::PointXYZ, pcl::Histogram<pcl::SGF5_SIZE> > sgf5;
    sgf5.setInputCloud (cloud);
    sgf5.setIndices(indicesptr);
    sgf5.compute (*sgf5s);

    for (size_t n = 0; n < pcl::SGF5_SIZE; ++n)
    {
      sgfs->points[0].histogram[feature_counter + n] = sgf5s->points[0].histogram[n];
    }
    feature_counter += pcl::SGF5_SIZE;

    // Print the result
    std::cout << "Feature SGF5: (";
    for (size_t idx = 0; idx < pcl::SGF5_SIZE - 1; ++idx)
    {
      std::cout << sgf5s->points[0].histogram[idx] << ", ";
    }
    std::cout << sgf5s->points[0].histogram[pcl::SGF5_SIZE - 1] << ")" << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////////////////
  // Feature 6
  if (IS_ELEMENT_OF(6, features))
  {
    pcl::PointCloud<pcl::Histogram<pcl::SGF6_SIZE> >::Ptr sgf6s (new pcl::PointCloud<pcl::Histogram<pcl::SGF6_SIZE> > ());
    pcl::SGF6Estimation<pcl::PointXYZ, pcl::Histogram<pcl::SGF6_SIZE> > sgf6;
    sgf6.setInputCloud (cloud);
    sgf6.setIndices(indicesptr);
    sgf6.compute (*sgf6s);

    for (size_t n = 0; n < pcl::SGF6_SIZE; ++n)
    {
      sgfs->points[0].histogram[feature_counter + n] = sgf6s->points[0].histogram[n];
    }
    feature_counter += pcl::SGF6_SIZE;

    // Print the result
    std::cout << "Feature SGF6: (" << sgf6s->points[0].histogram[0] << ", " << sgf6s->points[0].histogram[1] << ")" << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////////////////
  // Feature 7
  if (IS_ELEMENT_OF(7, features))
  {
    pcl::PointCloud<pcl::Histogram<pcl::SGF7_SIZE> >::Ptr sgf7s (new pcl::PointCloud<pcl::Histogram<pcl::SGF7_SIZE> > ());
    pcl::SGF7Estimation<pcl::PointXYZ, pcl::Histogram<pcl::SGF7_SIZE> > sgf7;
    sgf7.setInputCloud (cloud);
    sgf7.setIndices(indicesptr);
    sgf7.compute (*sgf7s);

    for (size_t n = 0; n < pcl::SGF7_SIZE; ++n)
    {
      sgfs->points[0].histogram[feature_counter + n] = sgf7s->points[0].histogram[n];
    }
    feature_counter += pcl::SGF7_SIZE;

    // Print the result
    std::cout << "Feature SGF7: (";
    for (size_t idx = 0; idx < pcl::SGF7_SIZE - 1; ++idx)
    {
      std::cout << sgf7s->points[0].histogram[idx] << ", ";
    }
    std::cout << sgf7s->points[0].histogram[pcl::SGF7_SIZE - 1] << ")" << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////////////////
  // Feature 8
  if (IS_ELEMENT_OF(8, features))
  {
    pcl::PointCloud<pcl::Histogram<pcl::SGF8_SIZE> >::Ptr sgf8s (new pcl::PointCloud<pcl::Histogram<pcl::SGF8_SIZE> > ());
    pcl::SGF8Estimation<pcl::PointXYZ, pcl::Histogram<pcl::SGF8_SIZE> > sgf8;
    sgf8.setInputCloud (cloud);
    sgf8.setIndices(indicesptr);
    sgf8.compute (*sgf8s);

    for (size_t n = 0; n < pcl::SGF8_SIZE; ++n)
    {
      sgfs->points[0].histogram[feature_counter + n] = sgf8s->points[0].histogram[n];
    }
    feature_counter += pcl::SGF8_SIZE;

    // Print the result
    std::cout << "Feature SGF8: (";
    for (size_t idx = 0; idx < pcl::SGF8_SIZE - 1; ++idx)
    {
      std::cout << sgf8s->points[0].histogram[idx] << ", ";
    }
    std::cout << sgf8s->points[0].histogram[pcl::SGF8_SIZE - 1] << ")" << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////////////////
  // Feature 9
  if (IS_ELEMENT_OF(9, features))
  {
    pcl::PointCloud<pcl::Histogram<pcl::SGF9_SIZE> >::Ptr sgf9s (new pcl::PointCloud<pcl::Histogram<pcl::SGF9_SIZE> > ());
    pcl::SGF9Estimation<pcl::PointXYZ, pcl::Histogram<pcl::SGF9_SIZE> > sgf9;
    sgf9.setInputCloud (cloud);
    sgf9.setIndices(indicesptr);
    sgf9.compute (*sgf9s);

    for (size_t n = 0; n < pcl::SGF9_SIZE; ++n)
    {
      sgfs->points[0].histogram[feature_counter + n] = sgf9s->points[0].histogram[n];
    }
    feature_counter += pcl::SGF9_SIZE;

    // Print the result
    std::cout << "Feature SGF9: " << sgf9s->points[0].histogram[0] << std::endl;
  }


  /////////////////////////////////////////////////////////////////////////////////////
  // Copy result into file

  pcl::io::savePCDFileASCII ("feature.pcd", *sgfs);
  //Eigen::Map<Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> > map = sgfs->getMatrixXfMap();

  std::ofstream writer;
  writer.open (feature_filename.c_str (), std::ios::out | std::ios::app);
  writer << label;
  for (size_t i = 0; i < feature_counter; ++i)
  {
    writer << " " << (i + 1) << ":" << sgfs->points[0].histogram[i];
  }
  writer << std::endl;
  writer.close ();


//  // Read the first line of a file
//  std::string line;
//  std::ifstream reader ("feature.pcd", std::ios::in);
//  if (reader.is_open ())
//  {
//    getline (reader, line);
//    reader.close ();
//  }
//
//  // parse the line
//  std::vector<std::string> v;
//  split (v, line, " ");
//  std::cout << "label: " << v[0] << std::endl;
//  for (size_t i = 1; i < v.size(); ++i)
//  {
//    std::vector<std::string> w;
//    split (w, v[i], ":");
//    std::cout << i << ":" << w[1] << std::endl;
//  }

  return (0);
}


void
split( std::vector<std::string> & theStringVector,  /* Altered/returned value */
       const std::string  & theString,
       const std::string  & theDelimiter)
{
  size_t start = 0, end = 0;

  while ( end != std::string::npos)
  {
    end = theString.find (theDelimiter, start);

    // If at end, use length=maxLength.  Else use length=end-start.
    theStringVector.push_back (theString.substr(start, (end == std::string::npos) ? std::string::npos : end - start));

    // If at end, use start=maxSize.  Else use start=end+delimiter.
    start = ((end > (std::string::npos - theDelimiter.size ())) ? std::string::npos : end + theDelimiter.size ());
  }
}

int bin2dec(const char *bin)
{
  int result=0;
  for(;*bin;bin++)
  {
    if((*bin!='0')&&(*bin!='1'))
      return -1;
    result=result*2+(*bin-'0');
    if(result<=0) return -1;
  }
  return result;
}

char* strrev( char* s )
{
  char  c;
  char* s0 = s - 1;
  char* s1 = s;

  /* Find the end of the string */
  while (*s1) ++s1;

  /* Reverse it */
  while (s1-- > ++s0)
    {
    c   = *s0;
    *s0 = *s1;
    *s1 =  c;
    }

  return s;
}
