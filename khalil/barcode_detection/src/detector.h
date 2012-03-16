/*
 * detector.h
 *
 *  Created on: Mar 14, 2012
 *      Author: banacer
 */

#ifndef DETECTOR_H_
#define DETECTOR_H_
#include <opencv-2.3.1/opencv/cv.h>
#include <opencv-2.3.1/opencv/highgui.h>
#include <opencv-2.3.1/opencv/cxcore.h>
#include <opencv-2.3.1/opencv2/core/core_c.h>
#include <opencv-2.3.1/opencv2/core/core.hpp>

namespace std
{

class detector
{
public:
  detector();
  virtual ~detector();


  void edgeImage(IplImage* );
  void switch_callback_h( int );
  void switch_callback_l( int );
  void dilationImage(IplImage* ,IplImage**);
  IplImage** absoluteDifference(IplImage** );
  void higherResponse(IplImage**,IplImage** );
  void delationErrosion(IplImage* , IplImage** );
  void selectBlob(IplImage* );
  int* createLine(int , int );
};

} /* namespace std */
#endif /* DETECTOR_H_ */
