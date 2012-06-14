/*
 * detector.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: banacer
 */

#include "detector.h"
#include "Line.h"
#include <c++/4.6/bits/stl_vector.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <eigen3/Eigen/Dense>
#include "squares.cpp"
#include "utils.h"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "ros/ros.h"

#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>



using namespace cv;
using namespace Eigen;
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
float maxDistance;
int minTransitions;
int maxTransitions;
int minROIlength;
ros::Publisher publisher;


void switch_callback_h( int position ){
        highInt = position;
}

void switch_callback_l( int position ){
        lowInt = position;
}
void edgeImage(IplImage** out,char* pic)
{
          // Kernel size
          int N = 7;
          // Set up images
          IplImage* img = cvLoadImage( pic, 0 );
          IplImage* img_b = cvCreateImage( cvSize(img->width+N-1,img->height+N-1), img->depth, img->nChannels );
          *out = cvCreateImage( cvGetSize(img_b), IPL_DEPTH_8U, img_b->nChannels );

          // Add convolution boarders
          CvPoint offset = cvPoint((N-1)/2,(N-1)/2);
          cvCopyMakeBorder(img, img_b, offset, IPL_BORDER_REPLICATE, cvScalarAll(0));

          // Edge Detection Variables
          int aperature_size = N;
          double lowThresh = 20;
          double highThresh = 40;

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
                  //cvShowImage(name, *out);
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
          temp[i][j] = 0;
      }

      //Allocation of array
      array = (int *) malloc(size * size * sizeof(int));

      //Initialization of array
      for(i = 0; i < size * size; i++)
        array[i] = 0;

      switch(angle)
      {
         case 0:
            j = size / 2;
            for(i = 0; i < size; i++)
               temp[j][i] = 1;

            break;
         case 45:
           for(i = 0; i < size; i++)
             temp[i][i] = 1;

           break;
         case 90:
           j = size/2;
            for(i = 0; i < size; i++)
              temp[i][j] = 1;

           break;
         case 135:
           for(i = 0; i < size; i++)
             temp[size-i-1][i] = 1;

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

void printLine(int* list,int size)
{
  for(int i = 0; i < size * size; i++)
  {
      cout << list[i] << " ";
  }
  cout << "\n";
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

    //0° line
    list = createLine(StructuringSize, 0);
    line0 = cvCreateStructuringElementEx(StructuringSize,StructuringSize,0,0,CV_SHAPE_CUSTOM,list);


    //45° line
    list = createLine(StructuringSize, 45);
    line45 = cvCreateStructuringElementEx(StructuringSize,StructuringSize,0,0,CV_SHAPE_CUSTOM,list);


    //90° line
    list = createLine(StructuringSize, 90);
    line90 = cvCreateStructuringElementEx(StructuringSize,StructuringSize,0,0,CV_SHAPE_CUSTOM,list);


    //135° line
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

     //2nd application
     cvDilate(img, *dest, line45,1);
     img = *dest;

     //3rd application
     cvDilate(img, *dest, line90,1);
     img = *dest;

     //4th application
     cvDilate(img, *dest, line135,1);
}

void findBlob(Mat& imgMat,vector<vector<Point> >& squares)
{
    imgMat = imread("final.png", 0);
    if( imgMat.empty() )
    {
        cout << "Couldn't load Image\n";
    }

    findSquares(imgMat, squares);
    Mat img;
    cv::cvtColor(imgMat,img,CV_GRAY2BGR,3);
    drawSquares(img, squares);
    imwrite("blob.jpg",img);

}

bool inRegion(int x, int y, vector<Point> roi)
{
    bool top = false;
    bool bottom = false;
    bool left = false;
    bool right = false;

    for(int i = 0; i < roi.size(); i++)
    {
        if(!top)
        {
            if(roi.at(i).y >= y)
              top = true;
        }

        if(!bottom)
        {
            if(roi.at(i).y <= y)
              bottom = true;
        }

        if(!left)
        {
            if(roi.at(i).x >= x)
              left = true;
        }

        if(!right)
        {
           if(roi.at(i).x <= x)
             right = true;
        }

        if(top & bottom & left & right) // This condition checks if the point has been proved to be in the region
            return true;
    }

    return false; //If it goes out of the loop, this means the point is not in the ROI
}

vector<Point> selectBlob(Mat img,vector<vector<Point> >& squares)
{
    vector<Point> empty;
    int topRoi = -1;
    double rate = 0;
    //cout << img.type();
    for(int i = 0; i < squares.size(); i++)
    {
        Point point;
        int numPixels = 0;
        int white = 0;
        double myRate = 0;
        for(uint row = 0; row < img.rows; row++)
        {
            for(uint col = 0; col < img.cols; col++)
            {
                //take a point from image

                int color = img.at<uchar>(row,col);

                //Check if is in the ROI (region of interest)

                if(inRegion(col,row,squares.at(i)))
                {
                    numPixels++;
                    //Mark if the point if black or white if in the blob
                    if(color == 255)
                      white++;
                }
            }
        }
        myRate = (double) ((double) white / (double) numPixels);

        if(rate <= myRate)
        {
            rate = myRate;
            topRoi = i;
        }
    }
    if(topRoi > -1)
      return squares.at(topRoi);
    std::cerr << "No squares found\n";
    return empty;
}



Mat rotateImg(Mat img,double angle)
{
    Mat rotated = utils::rotateImage(img,angle);
    imwrite("rotated.png",rotated);
    return rotated;
}


int blobMain(char *arg) {
  IplImage* edgeImg;

  //create edge image
  edgeImage(&edgeImg,arg);

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
  Mat imgMat;
  findBlob(imgMat,squares);


  //Select blob
  vector<Point> roi;
  roi = selectBlob(imgMat,squares);

  //Draw Selected Blob
  drawSquare(imgMat,roi);
  return 0;
}

vector<Point> findSquare(double**  data,int iSize, int jSize,int windowSize,float maxDistance,cv_bridge::CvImagePtr cv_bridge_ptr,int minROILength)
{
    Square* roi = new Square;
    Mat img = cv_bridge_ptr->image;
    cv::imwrite("result.jpg", img);
    vector<Square> list;
    double** duplicate = utils::copy2dimArray(data,iSize,jSize);
    roi->x = -1;
    roi->yStart = -1;
    roi->yEnd = -1;
    for(int i = 0; i < iSize; i++)
    {
        for(int j = 0; j < jSize; j++)
        {
            if(duplicate[i][j] != 0 && duplicate[i][j] != -1)
            {
                roi->x = j;
                roi->yStart = i;
                for(int k = i; k < iSize; k++)
                {
                    if(duplicate[k][j] == -1)
                    {
                        roi->yEnd = k-1;
                        if(k-1 - i > minROILength)
                          list.push_back(*roi);
                        roi = new Square;
                        roi->x = -1;
                        roi->yStart = -1;
                        roi->yEnd = -1;
                        break;
                    }
                    else
                        duplicate[k][j] = -1;
                }
            }
        }
    }

    double* avgList = new double[list.size()];
    int min = -1;
    for(int i = 0;i < list.size(); i++)
    {
        double count  = 0;
        for(int j = list.at(i).yStart; j < list.at(i).yEnd; j++)
        {
            if(data[j][list.at(i).x]  == 0)
              count+= maxDistance;
            else
              count += data[j][list.at(i).x];
        }
        count /= (double)(list.at(i).yEnd - list.at(i).yStart);
        avgList[i] = count;
        if(i == 0)
            min = i;
        else
        {
            if(count < avgList[i-1])
              min = i;
        }
    }
    /*for(int i = 0;i < list.size(); i++)
    {
        std::cout << "\nroi "<< i <<":\n\n";
        std::cout << "start y = " << list.at(i).yStart << ", x = " << list.at(i).x << "->" << (list.at(i).x + windowSize) << "\n";
        std::cout << "End   y = " << list.at(i).yEnd   << ", x = " << list.at(i).x << "->" << (list.at(i).x + windowSize) << "\n";
        std::cout << "avg Distance = " << avgList[i] << "\n";
    }*/
    vector<Point> square;
    std::cout << "hey 11\n";
    if(!list.empty())
    {

		if((list.at(min).yEnd - list.at(min).yStart < 30))
		{
		  std::cout << "hey 22\n";
		  std::cout << "\n\nCAUTION! cannot find good ROI\n";
		  cv_bridge::CvImage out_msg;
		  //out_msg.header   = img.header; // Same timestamp and tf frame as input image
		  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
		  out_msg.image    = img; // Your cv::Mat

		  publisher.publish(out_msg.toImageMsg());

		  vector<Point> p;
		  return p;
		}
		else
		{
			std::cout << "hey 33\n";
			//std:: cout <<"\nthe best roi\n\n";
			//std::cout << "start y = " << list.at(min).yStart << ", x = " << list.at(min).x << "->" << (list.at(min).x + windowSize) << "\n";
			//std::cout << "End   y = " << list.at(min).yEnd   << ", x = " << list.at(min).x << "->" << (list.at(min).x + windowSize) << "\n";
			std::cout << "BARCODE FOUND!\n";
			Point p1(list.at(min).x,list.at(min).yStart);
			Point p2(list.at(min).x + windowSize,list.at(min).yStart);
			Point p3(list.at(min).x,list.at(min).yEnd);
			Point p4(list.at(min).x + windowSize,list.at(min).yEnd);


			square.push_back(p1);
			square.push_back(p2);
			square.push_back(p4);
			square.push_back(p3);

			Mat myimg;
			cv::cvtColor(img,myimg,CV_GRAY2BGR,3);
			drawSquare(img,square);

			cv_bridge::CvImage out_msg;
			//out_msg.header   = img.header; // Same timestamp and tf frame as input image
			out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
			out_msg.image    = img; // Your cv::Mat

			publisher.publish(out_msg.toImageMsg());
			return square;
		}
    }
    else
    {
    	std::cout << "hey 44\n";
    	publisher.publish(cv_bridge_ptr->toImageMsg());

		vector<Point> p;
		return p;
    }


}

vector<Point> SlidingwindowDetection(cv_bridge::CvImagePtr cv_bridge_ptr,float maxDistance, int minTransitions, int maxTransitions, int minROILength)
{
	Mat img  = cv_bridge_ptr->image;
    int windowSize = 100;
    uint* interval1;
    uint* interval2;
    double** distanceValue = new double*[img.rows - 1];
    for(int i = 0; i < img.rows - 1; i++)
        distanceValue[i] =  new double[img.cols - windowSize + 1];

    int** transValue = new int*[img.rows];
        for(int i = 0; i < img.rows - 1; i++)
            transValue[i] =  new int[img.cols - windowSize + 1];

    double distance;
    for(int interval = 0; interval < img.cols - windowSize; interval++)
    {
        for(int i = 0; i < img.rows - 1; i++)
        {
            if(i == 0)
                interval1 = utils::getInterval(i,interval,interval + windowSize - 1,img);

            interval2 = utils::getInterval(i+1,interval,interval + windowSize - 1,img);
            int transitions = utils::binaryTransitions(interval1,windowSize);
            transValue[i][interval] = transitions;

            if(minTransitions <= transitions && transitions <= maxTransitions)
            {
                distance = utils::jeffriesDistance(interval1,interval2,windowSize);
                if(distance > maxDistance)
                  distanceValue[i][interval] = 0;
                else
                  distanceValue[i][interval] = distance;
            }
            else
              distanceValue[i][interval] = -1;


            delete [] interval1;
            interval1 = interval2;
        }

        delete [] interval1;
    }
    vector<Point> roi = findSquare(distanceValue,img.rows - 1,img.cols - windowSize + 1,windowSize,maxDistance,cv_bridge_ptr,minROILength);
    std::cout << "image analyzed\n";
    return roi;
}

void ImageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	cv_bridge::CvImagePtr cv_bridge_ptr;
	try
	{
		cv_bridge_ptr = cv_bridge::toCvCopy(msg_ptr, "mono8");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Error converting ROS Image");
		return;
	}

	vector<Point> roi = SlidingwindowDetection(cv_bridge_ptr,maxDistance,minTransitions,maxTransitions,minROIlength);
}

int main(int argc, char **argv) {

    /*Mat img = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    if(img.empty())
    {
        printf("Could not load image file: %s\n",argv[1]);
        return 0;
    }
	*/
    maxDistance = atof(argv[1]);
    minTransitions = atoi(argv[2]);
    maxTransitions = atoi(argv[3]);
    minROIlength = atoi(argv[4]);

    ros::init(argc, argv, "barcode_detection_node");
    ros::NodeHandle n("~");
    ros::AsyncSpinner spinner(1); // Use 4 threads
    spinner.start();

    publisher = n.advertise<sensor_msgs::Image>("detected_barcode", 1000);
    ros::Subscriber sub = n.subscribe("/image_raw", 50, ImageCallback);
    ros::spin();
}

