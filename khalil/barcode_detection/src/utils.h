/*
 * utils.h
 *
 *  Created on: Mar 26, 2012
 *      Author: banacer
 */

#ifndef UTILS_H_
#define UTILS_H_
#include "opencv2/core/core.hpp"
#include <c++/4.6/bits/stl_vector.h>
#include <eigen3/Eigen/Dense>

using namespace cv;
using namespace Eigen;

struct roi{
  int interval;
  int size;
  int angle;
  int rowStart;
  int rowEnd;
  double avgSimilarity;
  double avgTransitions;
};
struct intervalMerger{
  int start;
  int end;
  int yStart;
  int yEnd;
};

struct Square{
  int x;
  int yStart;
  int yEnd;
};
class utils
{
public:
  utils();
  static void copyArray(int* , int* ,int );
  static double hammingDistance(uint* ,uint* ,int );
  static double indenpendentDistance(uint* ,uint* ,int );
  static double transitionDiff(int* ,int* ,int );
  static int transitions(uint*, int size);
  static int binaryTransitions(uint* ,int );
  static uint* getInterval(int , int , int , Mat );
  static uint* getInterval(int , int , int , vector<int*> );
  static void printROI(roi );
  static bool lineEmpty(int* ,int );
  static Mat rotateImage(const Mat& , double );
  static intervalMerger* searchForInterval(vector<intervalMerger> ,int );
  static double euclidianDistance(uint* ,uint* ,int );
  static double manhattanDistance(uint* ,uint* ,int );
  static double jeffriesDistance(uint* ,uint* ,int );
  static double bhattacharyyaDistance(uint* ,uint* ,int );
  static double chiSquareDistance(uint* ,uint* ,int );
  static double klDivergenceDistance(uint* ,uint* ,int );
  static double** copy2dimArray(double** , int ,int );
  virtual ~utils();


private:
  static inline double euclidianDistance(const Eigen::VectorXf &, const Eigen::VectorXf &);
  static inline double manhattanDistance(const Eigen::VectorXf &, const Eigen::VectorXf &);
  static inline double jeffriesDistance(const Eigen::VectorXf &, const Eigen::VectorXf &);
  static inline double bhattacharyyaDistance(const Eigen::VectorXf &, const Eigen::VectorXf &);
  static inline double chiSquareDistance(const Eigen::VectorXf &, const Eigen::VectorXf &);
  static inline double klDivergenceDistance(const Eigen::VectorXf &, const Eigen::VectorXf &);
};



#endif /* UTILS_H_ */
