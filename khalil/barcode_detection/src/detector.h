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


  IplImage* edgeImage();
  void switch_callback_h( int );
  void switch_callback_l( int );
  IplImage** dilationImage(IplImage* );
  IplImage** absoluteDifference(IplImage** );
  iplImage* higherResponse(IplImage**);
  void createLine(cv::Mat ,int , int );
  void delationErrosion(IplImage* );
  void selectBlob(IplImage* );
};

} /* namespace std */
#endif /* DETECTOR_H_ */
