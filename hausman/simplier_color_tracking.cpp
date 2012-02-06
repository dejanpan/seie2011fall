
//simply color tracking

#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iomanip>


IplImage* GetThresholdedImage(IplImage* img, int color)
{
// Convert the image into an HSV image
IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
cvCvtColor(img, imgHSV, CV_BGR2HSV);

IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);
if (color==1)
// Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
cvInRangeS(imgHSV, cvScalar(20, 100, 100), cvScalar(30, 255, 255), imgThreshed);

if (color==2)
// Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
cvInRangeS(imgHSV, cvScalar(80, 100, 100), cvScalar(90, 255, 255), imgThreshed);

if (color==3)
// Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
cvInRangeS(imgHSV, cvScalar(80, 100, 100), cvScalar(90, 255, 255), imgThreshed);

if (color==4)
// Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
cvInRangeS(imgHSV, cvScalar(80, 100, 100), cvScalar(90, 255, 255), imgThreshed);

cvReleaseImage(&imgHSV);

return imgThreshed;
}

int main()
{
// Initialize capturing live feed from the camera
CvCapture* capture = 0;
capture = cvCaptureFromCAM(0);

// Couldn't get a device? Throw an error and quit
if(!capture)
    {
        printf("Could not initialize capturing...\n");
        return -1;
    }

// The two windows we'll be using
    cvNamedWindow("video");
cvNamedWindow("thresh yellow");
cvNamedWindow("thresh green");
//cvNamedWindow("thresh blue");
//cvNamedWindow("thresh red");


// This image holds the "scribble" data...
// the tracked positions of the ball
IplImage* imgScribble = NULL;

// An infinite loop
while(true)
    {
// Will hold a frame captured from the camera
IplImage* frame = 0;
frame = cvQueryFrame(capture);

// If we couldn't grab a frame... quit
        if(!frame)
            break;

// If this is the first frame, we need to initialize it
if(imgScribble == NULL)
{
imgScribble = cvCreateImage(cvGetSize(frame), 8, 3);
}

// Holds the yellow thresholded image (yellow = white, rest = black)
IplImage* imgYellowThresh = GetThresholdedImage(frame,1);
IplImage* imgGreenThresh = GetThresholdedImage(frame,2);
//IplImage* imgBlueThresh = GetThresholdedImage(frame,3);
//IplImage* imgRedThresh = GetThresholdedImage(frame,4);

// Calculate the moments to estimate the position of the ball
CvMoments *moments_yellow = (CvMoments*)malloc(sizeof(CvMoments));
CvMoments *moments_green = (CvMoments*)malloc(sizeof(CvMoments));
//CvMoments *moments_blue = (CvMoments*)malloc(sizeof(CvMoments));
//CvMoments *moments_red = (CvMoments*)malloc(sizeof(CvMoments));


cvMoments(imgYellowThresh, moments_yellow, 1);
cvMoments(imgGreenThresh, moments_green, 1);
//cvMoments(imgBlueThresh, moments_blue, 1);
//cvMoments(imgRedThresh, moments_red, 1);


// The actual moment values
double moment10y = cvGetSpatialMoment(moments_yellow, 1, 0);
double moment01y = cvGetSpatialMoment(moments_yellow, 0, 1);
double areay = cvGetCentralMoment(moments_yellow, 0, 0);

double moment10g = cvGetSpatialMoment(moments_green, 1, 0);
double moment01g = cvGetSpatialMoment(moments_green, 0, 1);
double areag = cvGetCentralMoment(moments_green, 0, 0);

//double moment10b = cvGetSpatialMoment(moments_blue, 1, 0);
//double moment01b = cvGetSpatialMoment(moments_blue, 0, 1);
//double areab = cvGetCentralMoment(moments_blue, 0, 0);

//double moment10r = cvGetSpatialMoment(moments_red, 1, 0);
//double moment01r = cvGetSpatialMoment(moments_red, 0, 1);
//double arear = cvGetCentralMoment(moments_red, 0, 0);

// Holding the last and current ball positions
static int posXy = 0;
static int posYy = 0;

int lastXy = posXy;
int lastYy = posYy;

posXy = moment10y/areay;
posYy = moment01y/areay;

// Print it out for debugging purposes
printf("position yellow (%d,%d)\n", posXy, posYy);



static int posXg = 0;
static int posYg = 0;

int lastXg = posXg;
int lastYg = posYg;

posXg = moment10g/areag;
posYg = moment01g/areag;

// Print it out for debugging purposes
printf("position green (%d,%d)\n", posXg, posYg);


//static int posXb = 0;
//static int posYb = 0;
//
//int lastXb = posXb;
//int lastYb = posYb;
//
//posXb = moment10b/areab;
//posYb = moment01b/areab;
//
//// Print it out for debugging purposes
//printf("position blue (%d,%d)\n", posXb, posYb);


//static int posXr = 0;
//static int posYr = 0;
//
//int lastXr = posXr;
//int lastYr = posYr;
//
//posXr = moment10r/arear;
//posYr = moment01r/arear;
//
//// Print it out for debugging purposes
//printf("position red (%d,%d)\n", posXr, posYr);

// We want to draw a line only if its a valid position
if(lastXy>0 && lastYy>0 && posXy>0 && posYy>0)
{
// Draw a yellow line from the previous point to the current point
cvLine(imgScribble, cvPoint(posXy, posYy), cvPoint(lastXy, lastYy), cvScalar(0,255,255), 5);
}

if(lastXg>0 && lastYg>0 && posXg>0 && posYg>0)
{
// Draw a yellow line from the previous point to the current point
cvLine(imgScribble, cvPoint(posXg, posYg), cvPoint(lastXg, lastYg), cvScalar(0,255,0), 5);
}


//if(lastXb>0 && lastYb>0 && posXb>0 && posYb>0)
//{
//// Draw a yellow line from the previous point to the current point
//cvLine(imgScribble, cvPoint(posXg, posYg), cvPoint(lastXg, lastYg), cvScalar(0,0,255), 5);
//}
//
//if(lastXr>0 && lastYr>0 && posXr>0 && posYr>0)
//{
//// Draw a yellow line from the previous point to the current point
//cvLine(imgScribble, cvPoint(posXr, posYr), cvPoint(lastXr, lastYr), cvScalar(255,0,0), 5);
//}

// Add the scribbling image and the frame... and we get a combination of the two
cvAdd(frame, imgScribble, frame);
cvShowImage("thresh yellow", imgYellowThresh);
cvShowImage("thresh green", imgGreenThresh);
//cvShowImage("thresh blue", imgBlueThresh);
//cvShowImage("thresh red", imgRedThresh);
cvShowImage("video", frame);

// Wait for a keypress
int c = cvWaitKey(10);
if(c!=-1)
{
// If pressed, break out of the loop
            break;
}

// Release the thresholded image... we need no memory leaks.. please
cvReleaseImage(&imgYellowThresh);
cvReleaseImage(&imgGreenThresh);
//cvReleaseImage(&imgBlueThresh);
//cvReleaseImage(&imgRedThresh);


delete moments_yellow;
delete moments_green;
//delete moments_blue;
//delete moments_red;

    }

// We're done using the camera. Other applications can now use it
cvReleaseCapture(&capture);
    return 0;
}

