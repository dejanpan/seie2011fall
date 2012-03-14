/*
 * detector.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: banacer
 */

#include "detector.h"
#include <opencv-2.3.1/opencv/cv.h>
#include <opencv-2.3.1/opencv/highgui.h>
#include <opencv-2.3.1/opencv/cxcore.h>
#include <opencv-2.3.1/opencv2/core/core_c.h>
#include <opencv-2.3.1/opencv2/core/core.hpp>




namespace std
{

detector::detector()
{
  // TODO Auto-generated constructor stub

}

detector::~detector()
{
  // TODO Auto-generated destructor stub
}

} /* namespace std */
int high_switch_value = 0;
int highInt = 0;
int low_switch_value = 0;
int lowInt = 0;


void switch_callback_h( int position ){
        highInt = position;
}

void switch_callback_l( int position ){
        lowInt = position;
}
IplImage* edgeImage()
{
          const char* name = "Edge Detection Window";

          // Kernel size
          int N = 7;

          // Set up images
          IplImage* img = cvLoadImage( "img.jpg", 0 );
          IplImage* img_b = cvCreateImage( cvSize(img->width+N-1,img->height+N-1), img->depth, img->nChannels );
          IplImage* out = cvCreateImage( cvGetSize(img_b), IPL_DEPTH_8U, img_b->nChannels );

          // Add convolution boarders
          CvPoint offset = cvPoint((N-1)/2,(N-1)/2);
          cvCopyMakeBorder(img, img_b, offset, IPL_BORDER_REPLICATE, cvScalarAll(0));

          // Make window
          cvNamedWindow( name, 1 );

          // Edge Detection Variables
          int aperature_size = N;
          double lowThresh = 20;
          double highThresh = 40;

          // Create trackbars
          cvCreateTrackbar( "High", name, &high_switch_value, 4, switch_callback_h );
          cvCreateTrackbar( "Low", name, &low_switch_value, 4, switch_callback_l );

          while( 1 ) {
                  switch( highInt ){
                          case 0:
                                  highThresh = 200;
                                  break;
                          case 1:
                                  highThresh = 400;
                                  break;
                          case 2:
                                  highThresh = 600;
                                  break;
                          case 3:
                                  highThresh = 800;
                                  break;
                          case 4:
                                  highThresh = 1000;
                                  break;
                  }
                  switch( lowInt ){
                          case 0:
                                  lowThresh = 0;
                                  break;
                          case 1:
                                  lowThresh = 100;
                                  break;
                          case 2:
                                  lowThresh = 200;
                                  break;
                          case 3:
                                  lowThresh = 400;
                                  break;
                          case 4:
                                  lowThresh = 600;
                                  break;
                  }

                  // Edge Detection
                  cvCanny( img_b, out, lowThresh*N*N, highThresh*N*N, aperature_size );
                  cvShowImage(name, out);

                  if( cvWaitKey( 15 ) == 27 )
                          break;

                  return out;
          }

          // Release
          cvReleaseImage( &img );
          cvReleaseImage( &img_b );
          cvReleaseImage( &out );
          cvDestroyWindow( name );
}

IplImage* dilationImage(IplImage* srcImg)
{
    //Creating the dilation image structure

    IplImage* imgList[4];
    int i;
    //initialiation of imgList
    for(i = 0; i < 4; i++)
      imgList[i] = NULL;

    //creating the structural element
    cv::Mat line0 = line0(20,20,CV_8U,cv::Scalar(1));
    cv::Mat line45 = line45(20,20,CV_8U,cv::Scalar(1));
    cv::Mat line90 = line90(20,20,CV_8U,cv::Scalar(1));
    cv::Mat line135 = line135(20,20,CV_8U,cv::Scalar(1));

    createLine(line0,20, 0);
    createLine(line45,20, 45);
    createLine(line90,20, 90);
    createLine(line135,20, 135);

    //applying the dilation
    cv::Dilate(srcImg, imgList[0], &line0, Point(-1, -1), 1,BORDER_CONSTANT,morphologyDefaultBorderValue());
    cv::Dilate(srcImg, imgList[1], &line45, Point(-1, -1), 1,BORDER_CONSTANT,morphologyDefaultBorderValue());
    cv::Dilate(srcImg, imgList[2], &line90, Point(-1, -1), 1,BORDER_CONSTANT,morphologyDefaultBorderValue());
    cv::Dilate(srcImg, imgList[3], &line135,Point(-1, -1), 1,BORDER_CONSTANT,morphologyDefaultBorderValue());


    return imgList;
}

IplImage** absoluteDifference(IplImage* ImgList[])
{
    IplImage* absoluteList[2] = NULL;

    //initialisation of absoluteList
    absoluteList[0] = NULL;
    absoluteList[1] = NULL;

    //computing the absolute difference
    cvAbsDiff(ImgList[0], ImgList[2], absoluteList[0]);
    cvAbsDiff(ImgList[1], ImgList[3], absoluteList[1]);

    return absoluteList;
}

iplImage* higherResponse(IplImage* absoluteList[])
{
  IplImage* maxResponseImg;
  cv::max(absoluteList[0], absoluteList[1],maxResponseImg);
  return maxResponseImg;
}

void createLine(cv::Mat structuralElement,int size, int angle)
{
  int i,j;
  switch(angle)
  {
    case 0:
      j = size/2;
      for(i = 0; i < size; i++)
        structuralElement.at<uchar>(j,i) = 0;
      break;
    case 45:
      for(i = 0; i < size; i++)
        structuralElement.at<uchar>(i,i) = 0;

      break;
    case 90:
      j = size/2;
       for(i = 0; i < size; i++)
         structuralElement.at<uchar>(i,j) = 0;
      break;
    case 135:
      for(i = 0; i < size; i++)
        structuralElement.at<uchar>(size - i,i) = 0;
      break;
  }
}

IplImage* delationErrosion(IplImage* img)
{
     IplImage* dest = NULL;
     cv::Mat line0 = line0(20,20,CV_8U,cv::Scalar(1));
     cv::Mat line45 = line45(20,20,CV_8U,cv::Scalar(1));
     cv::Mat line90 = line90(20,20,CV_8U,cv::Scalar(1));
     cv::Mat line135 = line135(20,20,CV_8U,cv::Scalar(1));

     createLine(line0,20, 0);
     createLine(line45,20, 45);
     createLine(line90,20, 90);
     createLine(line135,20, 135);

     //1st application
     cv::Dilate(img, dest, &line0, Point(-1, -1), 1,BORDER_CONSTANT,morphologyDefaultBorderValue());
     img = dest;
     cv::Erode(img, dest, &line0, Point(-1, -1), 1,BORDER_CONSTANT,morphologyDefaultBorderValue());
     img = dest;
     //2nd application
     cv::Dilate(img, dest, &line0, Point(-1, -1), 1,BORDER_CONSTANT,morphologyDefaultBorderValue());
     img = dest;
     cv::Erode(img, dest, &line0, Point(-1, -1), 1,BORDER_CONSTANT,morphologyDefaultBorderValue());
     img = dest;
     //3rd application
     cv::Dilate(img, dest, &line0, Point(-1, -1), 1,BORDER_CONSTANT,morphologyDefaultBorderValue());
     img = dest;
     cv::Erode(img, dest, &line0, Point(-1, -1), 1,BORDER_CONSTANT,morphologyDefaultBorderValue());
     img = dest;
     //4th application
     cv::Dilate(img, dest, &line0, Point(-1, -1), 1,BORDER_CONSTANT,morphologyDefaultBorderValue());
     img = dest;
     cv::Erode(img, dest, &line0, Point(-1, -1), 1,BORDER_CONSTANT,morphologyDefaultBorderValue());
     img = dest;
     return img;
}

void selectBlob(IplImage* img)
{

}


