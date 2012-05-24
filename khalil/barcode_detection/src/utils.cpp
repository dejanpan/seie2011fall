/*
 * utils.cpp
 *
 *  Created on: Mar 26, 2012
 *      Author: banacer
 */
#include <iostream>
#include "utils.h"
#include <c++/4.6/bits/stl_vector.h>
#include "opencv2/opencv.hpp"


utils::utils()
{
  // TODO Auto-generated constructor stub

}

utils::~utils()
{
  // TODO Auto-generated destructor stub
}

void utils::copyArray(int arr1[], int arr2[],int size)
{
  for(int i = 0; i < size; i++)
    arr1[i] = arr2[i];
}

double utils::hammingDistance(uint* arr1,uint* arr2,int size)
{
    double count = 0;
    for(int i = 0; i < size; i++)
    {
        if(arr1[i] != arr2[i])
          count++;
    }
    return count;
}
double utils::indenpendentDistance(uint* arr1,uint* arr2,int size)
{
    double countWhiteInterval1 = 0;
    double countBlackInterval1 = 0;
    double countWhiteInterval2 = 0;
    double countBlackInterval2 = 0;
    for(int i = 0; i < size; i++)
    {
        if(arr1[i] == 0)
          countWhiteInterval1++;

        if(arr1[i] == 255)
          countBlackInterval1++;

        if(arr2[i] == 0)
          countWhiteInterval2++;

        if(arr2[i] == 255)
          countBlackInterval2++;

    }
    return (double) abs(countBlackInterval1 - countBlackInterval2) + abs(countWhiteInterval1 - countWhiteInterval2);
}

int utils::transitions(uint* arr,int size)
{
    int color = arr[0];
    int transitions = 0;
    for(int i = 1; i < size; i++)
    {
        if(arr[i] != color)
        {
          transitions++;
          color = arr[i];
        }
    }
    return transitions;
}

int utils::binaryTransitions(uint* arr,int size)
{
    uint* binInterval = new uint[size];
    for(int i = 0; i < size; i++)
    {
        if(arr[i] < 122)
          binInterval[i] = 0;
        else
          binInterval[i] = 1;
    }
    int result = transitions(binInterval,size);
    delete [] binInterval;
    return result;
}

uint* utils::getInterval(int y, int xBeg, int xEnd, Mat img)
{
   uint* values = new uint [xEnd - xBeg + 1];
   int count = 0;
   for(int i = xBeg; i <= xEnd; i++)
   {
     values[count] = (uint) img.at<uchar>(y,i);
     /*if(((uint) img.at<uchar>(y,i)) < 122)
       values[count] = 0;
     else
       values[count] = 1;*/
     count++;
   }
   return values;
}
uint* utils::getInterval(int y, int xBeg, int xEnd, vector<int*> list)
{
     uint* values = new uint [xEnd - xBeg + 1];
     int count = 0;
     for(int i = xBeg; i <= xEnd; i++)
     {
       values[count] = list.at(y)[i];
       count++;
     }
     return values;
}

void utils::printROI(roi r)
{
    std::cout << "start: " << r.rowStart << ", end: " << r.rowEnd << ", size: "<< (r.rowEnd - r.rowStart) <<", avg similarity: " << r.avgSimilarity << ", avg transitions: " << r.avgTransitions << "\n";
}

bool utils::lineEmpty(int* line, int size)
{
    for(int i = 0; i < size; i++)
    {
      if(line[i] != -1)
        return false;
    }
    return true;
}

Mat utils::rotateImage(const Mat& source,double angle)
{
      Point2f src_center(source.cols/2.0F, source.rows/2.0F);
      Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
      Mat dst;
      warpAffine(source, dst, rot_mat, source.size());
      return dst;
}

intervalMerger* utils::searchForInterval(vector<intervalMerger> list,int interval)
{
    for(int i = 0; i < list.size(); i++)
    {
        if(list.at(i).start <= interval || (list.at(i).start < interval && interval <= list.at(i).end))
            return &list.at(i);
    }
}

double utils::euclidianDistance(uint* arr1,uint* arr2,int size)
{
  Eigen::VectorXf v1(size);
  Eigen::VectorXf v2(size);

  for(int i = 0; i < size; i++)
  {
      v1[i] = arr1[i];
      v2[i] = arr2[i];
  }

  return euclidianDistance(v1,v2);
}

double utils::manhattanDistance(uint* arr1,uint* arr2,int size)
{
    Eigen::VectorXf v1(size);
    Eigen::VectorXf v2(size);

    for(int i = 0; i < size; i++)
    {
        v1[i] = arr1[i];
        v2[i] = arr2[i];
    }

    return manhattanDistance(v1,v2);
}

double utils::jeffriesDistance(uint* arr1,uint* arr2,int size)
{
    Eigen::VectorXf v1(size);
    Eigen::VectorXf v2(size);

    for(int i = 0; i < size; i++)
    {
        v1[i] = arr1[i];
        v2[i] = arr2[i];
    }

    return jeffriesDistance(v1,v2);
}

double utils::bhattacharyyaDistance(uint* arr1,uint* arr2,int size)
{
    Eigen::VectorXf v1(size);
    Eigen::VectorXf v2(size);

    for(int i = 0; i < size; i++)
    {
        v1[i] = arr1[i];
        v2[i] = arr2[i];
    }

    return bhattacharyyaDistance(v1,v2);
}

double utils::chiSquareDistance(uint* arr1,uint* arr2,int size)
{
  Eigen::VectorXf v1(size);
      Eigen::VectorXf v2(size);

      for(int i = 0; i < size; i++)
      {
          v1[i] = arr1[i];
          v2[i] = arr2[i];
      }

      return chiSquareDistance(v1,v2);
}

double utils::klDivergenceDistance(uint* arr1,uint* arr2,int size)
{
  Eigen::VectorXf v1(size);
        Eigen::VectorXf v2(size);

        for(int i = 0; i < size; i++)
        {
            v1[i] = arr1[i];
            v2[i] = arr2[i];
        }

        return klDivergenceDistance(v1,v2);
}

double** utils::copy2dimArray(double** arr, int iSize,int jSize)
{
    double** toReturn = new double*[iSize];

    for(int i = 0; i < iSize; i++)
        toReturn[i] = new double[jSize];

    for(int i = 0; i < iSize; i++)
    {
      for(int j = 0; j < jSize; j++)
          toReturn[i][j] = arr[i][j];
    }
    return toReturn;
}



//PRIVATE METHODS
inline double utils::euclidianDistance(const Eigen::VectorXf &v1, const Eigen::VectorXf &v2)
{
  return (v1-v2).norm();
}

inline double utils::manhattanDistance(const Eigen::VectorXf &v1, const Eigen::VectorXf &v2)
{

  return (v1-v2).cwiseAbs().sum();
}

inline double utils::jeffriesDistance(const Eigen::VectorXf &v1, const Eigen::VectorXf &v2)
{
  /*std::cerr << "v1: " << v1 << "\n";
  std::cerr << "v2: " << v2 << "\n";*/

  return std::sqrt((v1.array().sqrt()-v2.array().sqrt()).square().sum());
}

inline double utils::bhattacharyyaDistance(const Eigen::VectorXf &v1, const Eigen::VectorXf &v2)
{
  double sum = 0;
  for(int i=0;i<v1.rows();i++)
  {
    sum += sqrt(abs(v1[i]-v2[i]));
  }
  //std::cerr<<"returning: "<<-log(sum);
  return -log (sum) ;
}

inline double utils::chiSquareDistance(const Eigen::VectorXf &v1, const Eigen::VectorXf &v2)
{
  double sum=0;
  for(int i=0;i<v1.rows();i++)
  {
    if(v1[i]+v2[i] == 0)
      continue;
    sum +=(double)((v1[i]-v2[i])*(v1[i]-v2[i]))/(v1[i]+v2[i]);

  }
  return sum;
}

inline double utils::klDivergenceDistance(const Eigen::VectorXf &v1, const Eigen::VectorXf &v2)
{
  double sum = 0;
  for(int i=0;i<v1.rows();i++)
  {
    if(v2[i]==0)
      continue;
    sum += (v1[i]-v2[i])*log(v1[i]/v2[i]);
  }
  return sum;
}
