/*
 * detector.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: banacer
 */

#include "detector.h"
#include <c++/4.6/bits/stl_vector.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "squares.cpp"



using namespace cv;
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
//Structuring elements
IplConvKernel* line0;
IplConvKernel* line45;
IplConvKernel* line90;
IplConvKernel* line135;



void switch_callback_h( int position ){
        highInt = position;
}

void switch_callback_l( int position ){
        lowInt = position;
}
void edgeImage(IplImage** out,char* pic)
{
          const char* name = "Edge Detection Window";
          // Kernel size
          int N = 7;
          // Set up images
          IplImage* img = cvLoadImage( pic, 0 );
          IplImage* img_b = cvCreateImage( cvSize(img->width+N-1,img->height+N-1), img->depth, img->nChannels );
          *out = cvCreateImage( cvGetSize(img_b), IPL_DEPTH_8U, img_b->nChannels );

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
                  cvCanny( img_b, *out, lowThresh*N*N, highThresh*N*N, aperature_size );
                  cvShowImage(name, *out);
                  return;
          }
}

int* createLine(int size, int angle)
{
      int i,j,count;
      int* array;
      //creation of temporary 2-dimensional array
      int temp[size][size];

      //Initialization of temporary 2-dimensional array
      for(i = 0; i < size; i++)
      {
        for(j = 0; j < size; j++)
          temp[i][j] = 1;
      }

      //Allocation of array
      array = (int *) malloc(size * size * sizeof(int));

      //Initialization of array
      for(i = 0; i < size * size; i++)
        array[i] = 1;

      switch(angle)
      {
         case 0:
            j = size/2;
            for(i = 0; i < size; i++)
               temp[j][i] = 0;

            break;
         case 45:
           for(i = 0; i < size; i++)
             temp[0][0] = 0;

           break;
         case 90:
           j = size/2;
            for(i = 0; i < size; i++)
              temp[i][j] = 0;

           break;
         case 135:
           for(i = 0; i < size; i++)
             temp[size-i][i] = 0;

           break;
       }
      //Conversion from 2-dimensional array to 1-dimensional array
      count = 0;
      for(i = 0; i < size; i++)
      {
        for(j = 0; j < size; j++)
        {
            array[count] = temp[i][j];
            count++;
        }
      }
      return array;

}

void dilationImage(IplImage* srcImg,IplImage* imgList[])
{
    int* list;
    int StructuringSize = 2;
    //Creating the dilation image structure

    //initialiation of imgList
    for(int i = 0; i < 4; i++)
        imgList[i] = cvCreateImage( cvSize(srcImg->width,srcImg->height), srcImg->depth, srcImg->nChannels );

    //creating the structural element

    list = createLine(StructuringSize, 0);
    line0 = cvCreateStructuringElementEx(StructuringSize,StructuringSize,0,0,CV_SHAPE_CUSTOM,list);

    list = createLine(StructuringSize, 45);
    line45 = cvCreateStructuringElementEx(StructuringSize,StructuringSize,0,0,CV_SHAPE_CUSTOM,list);

    list = createLine(StructuringSize, 90);
    line90 = cvCreateStructuringElementEx(StructuringSize,StructuringSize,0,0,CV_SHAPE_CUSTOM,list);

    list = createLine(StructuringSize, 135);
    line135 = cvCreateStructuringElementEx(StructuringSize,StructuringSize,0,0,CV_SHAPE_CUSTOM,list);

    //applying the dilation
    cvDilate(srcImg, imgList[0], line0, 1);
    cvDilate(srcImg, imgList[1], line45, 1);
    cvDilate(srcImg, imgList[2], line90, 1);
    cvDilate(srcImg, imgList[3], line135, 1);

}

IplImage** absoluteDifference(IplImage* ImgList[])
{
    IplImage** absoluteList  = (IplImage**) calloc(2,sizeof(IplImage*));

    //Initialization of absoluteList
    absoluteList[0] = cvCreateImage( cvSize(ImgList[0]->width,ImgList[0]->height), ImgList[0]->depth, ImgList[0]->nChannels );
    absoluteList[1] = cvCreateImage( cvSize(ImgList[0]->width,ImgList[0]->height), ImgList[0]->depth, ImgList[0]->nChannels );

    //computing the absolute difference
    cvAbsDiff(ImgList[0], ImgList[2], absoluteList[0]);
    cvAbsDiff(ImgList[1], ImgList[3], absoluteList[1]);

    return absoluteList;
}

void higherResponse(IplImage* absoluteList[],IplImage** maxResponseImg)
{
  *maxResponseImg = cvCreateImage( cvSize(absoluteList[0]->width,absoluteList[0]->height), absoluteList[0]->depth, absoluteList[0]->nChannels );
  cvMax(absoluteList[0], absoluteList[1],*maxResponseImg);

}

void delationErrosion(IplImage* img, IplImage** dest)
{
     *dest = cvCreateImage( cvSize(img->width,img->height), img->depth, img->nChannels );
     //1st application
     cvDilate(img, *dest, line0,1);
     img = *dest;
     //cvErode(img, *dest, line0,1);
     //img = *dest;
     //2nd application
     cvDilate(img, *dest, line45,1);
     img = *dest;
     //cvErode(img, *dest, line45,1);
     //img = *dest;
     //3rd application
     cvDilate(img, *dest, line90,1);
     img = *dest;
     //cvErode(img, *dest, line90,1);
     //img = *dest;
     //4th application
     cvDilate(img, *dest, line135,1);
     img = *dest;
     //cvErode(img, *dest, line135,1);

}

void selectBlob(IplImage* img)
{

}


int main(int argc, char **argv) {
  IplImage* edgeImg;

  //create edge image
  edgeImage(&edgeImg,argv[1]);

  cvSaveImage("edge.png",edgeImg);

  //create dilation images
  IplImage* imgList[4];
  dilationImage(edgeImg,imgList);

  cvSaveImage("dilation1.png",imgList[0]);
  cvSaveImage("dilation2.png",imgList[1]);
  cvSaveImage("dilation3.png",imgList[2]);
  cvSaveImage("dilation4.png",imgList[3]);

  //Absolute Difference images
  IplImage** absoluteDifferenceList;
  absoluteDifferenceList = absoluteDifference(imgList);

  cvSaveImage("absolute1.png",absoluteDifferenceList[0]);
  cvSaveImage("absolute2.png",absoluteDifferenceList[1]);

  //Higher response image
  IplImage* higherResponseImg;
  higherResponse(absoluteDifferenceList,&higherResponseImg);

  cvSaveImage("higherResponse.png",higherResponseImg);

  //Dilation and Erosion image
  IplImage* final;
  delationErrosion(higherResponseImg,&final);

  cvSaveImage("final.png",final);

  //Blob detection
  vector<vector<Point> > squares;
  Mat imgMat = imread("final.png", 1);
  if( imgMat.empty() )
  {
      cout << "Couldn't load Image\n";
  }

  findSquares(imgMat, squares);
  drawSquares(imgMat, squares);
  int c = waitKey();
  if( (char)c == 27 )
        return 0;
}



