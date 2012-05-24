/*
 * Line.cpp
 *
 *  Created on: Mar 22, 2012
 *      Author: banacer
 */

#include "Line.h"
#include <math.h>
#include <iostream>
using namespace cv;
Line::Line()
{
  // TODO Auto-generated constructor stub

}
Line::Line(int size, int myangle)
{
    // TODO Auto-generated constructor stub
    values = (int* )calloc(size,sizeof(int));
    points = (struct myPoint*) calloc(size,sizeof(struct myPoint));
    mover = (struct myPoint*) calloc(size,sizeof(struct myPoint));
    angle = myangle;
    Line::size = size;
    location = 0;
    //Initialization of values

    for(int i = 0; i < size; i++)
    {
        values[i]   = -1;
        points[i].x = -1;
        points[i].y = -1;
    }
    //Contruction of mover
    constructMover();
}

Line::~Line()
{
  // TODO Auto-generated destructor stub
}
int* Line::getValues()
{
  return values;
}

int Line::getValue(int index)
{
    return values[index];
}

void Line::setValue(int value, int index)
{
    values[index] = value;
}

struct myPoint Line::getPoint(int index)
{
    return points[index];
}

void Line::setPoint(struct myPoint point,int index)
{
    points[index] = point;
}

int Line::getAngle()
{
  return angle;
}

void Line::setAngle(int angle)
{
    Line::angle = angle;
    Line::constructMover();
}

int Line::getCenterPoint()
{
    return Line::centerPoint;
}

void Line::setCenterPoint(int point)
{
    Line::centerPoint = point;
}

int Line::stepMove()
{
    double x, y;
    double count = 1;

    updated = false;
}

int Line::getSize()
{
  return size;
}

bool Line::isUpdated()
{
  return updated;
}

void Line::update(Mat img)
{
    for(int i = 0; i < size; i++)
    {
        if(points[i].x < img.cols && points[i].y < img.rows)
          values[i] = img.at<uint>(points[i].x,points[i].y);
        else
          values[i] = -1;
    }
}

bool Line::graspsImage(Mat img)
{
    bool minPointExist = false;
    bool maxPointExist = false;
    int ymax = img.rows - 1;
    int xmax = img.cols - 1;
    int xmin = 0;
    int ymin = 0;
    for(int i = 0; i < size; i++)
    {
        //maxPoint existence condition
        if(Line::points[i].y >= ymax || Line::points[i].x >= xmax)
            maxPointExist = true;
        //minPoint existence condition
        if(Line::points[i].y <= ymin || Line::points[i].x <= xmin)
            minPointExist = true;

        //The following are conditions where line touches image only on edges
              if(Line::points[i].x == xmin && Line::points[i].y == ymax)
                  return true;
              if(Line::points[i].x == xmin && Line::points[i].y == ymin)
                  return true;
              if(Line::points[i].x == xmax && Line::points[i].y == ymax)
                  return true;
              if(Line::points[i].x == xmax && Line::points[i].y == ymin)
                  return true;
    }

    if(minPointExist && maxPointExist)
        return true;
    return false;

}

bool Line::pointInImage(Mat img)
{
    struct myPoint p = getPoint(Line::centerPoint);
    if((0 <= p.x && p.x < img.cols) && (0 <= p.y && p.y < img.rows))
      return true;
    return false;
}

void Line::chooseStartingPoint(Mat img)
{
    bool correct = true;
    for(int i = 0; i < size; i++)
    {
        correct = true;

        constructLine(i,img.rows);
        setCenterPoint(i);
        while(pointInImage(img))
        {
            if(!graspsImage(img))
            {
                correct = false;
                break;
            }
            moveLine(location+1);
        }
        if(correct)
        {
            moveLine(0); //Move line to the first location
            return;
        }
    }

    std::cerr << "Line is not long enough to grasp all image with the angle "<< angle;
}

void Line::constructMover()
{
    const double PI = 3.141592;
    mover[0].x = 0;
    mover[0].y = 0;
    double myAngle = (double)angle - 90;
    double radianAngle = (PI * myAngle) / 180;
    int x;
    int y;
    bool xFixed; //This indicate if x is fixed or y is fixed
    int count;

    if(  -90 < myAngle && myAngle < -45)
    {
        x = 1;
        y = (int) (((double) -x) * tan(radianAngle));
        xFixed = true;
    }

    if( -45 < myAngle && myAngle < 0)
    {
        y = 1;
        x = (int) (((double) y) / tan(radianAngle));
        xFixed = false;
    }

    if( 0 < myAngle && myAngle < 45)
    {
        y = 1;
        x = (int) (((double) y) / tan(radianAngle));
        xFixed = false;
    }

    if( 45 < myAngle && myAngle < 90)
    {
        x = 1;
        y = (int) (((double) x) * tan(radianAngle));
        xFixed = true;
    }

    if(myAngle == -90 ||myAngle == -45 || myAngle == 0 || myAngle == 45)
    {
        if(myAngle == -90)
        {
            for(int i = 1; i < size; i++)
            {
                mover[i].x = mover[i - 1].x;
                mover[i].y = mover[i - 1].y - 1;
            }
        }

        if(myAngle == -45)
        {
            for(int i = 1; i < size; i++)
            {
                mover[i].x = mover[i - 1].x - 1;
                mover[i].y = mover[i - 1].y + 1;
            }
        }

        if(myAngle == 0)
        {
            for(int i = 1; i < size; i++)
            {
                mover[i].x = mover[i - 1].x + 1;
                mover[i].y = mover[i - 1].y;
            }
        }
        if(myAngle == 45)
        {
            for(int i = 1; i < size; i++)
            {
                mover[i].x = mover[i - 1].x + 1;
                mover[i].y = mover[i - 1].y + 1;
            }

        }
    }
    else
    {
        y = abs(y);
        x = abs(x);
        count = 1;
        if(xFixed)
        {
            while(count < size)
            {
                if(angle < 0)
                    mover[count].y = mover[count - 1].y - 1;
                else
                    mover[count].y = mover[count - 1].y + 1;

                mover[count].x = mover[count - 1].x;

                if(count % y == 0)
                    mover[count].x += 1;
                count++;
            }
        }
        else
        {
            while(count < size)
            {
                mover[count].x = mover[count - 1].x + 1;
                mover[count].y = mover[count - 1].y;

                if(count % x == 0)
                {
                  if(angle >= 0)
                      mover[count].y += 1;
                  else
                      mover[count].y -= 1;
                }
                count++;
            }
        }
    }
}

void Line::constructLine(int start, int yMax)
{
    const double PI = 3.141592;
    points[start].x = 0;
    points[start].y = 0;
    double myAngle = (double) angle;
    double radianAngle = (PI * myAngle) / 180;

    if(myAngle <= 90) // IF angle is less than 90, the line should go from up to down
      points[start].y = yMax -1;

    int x;
    int y;
    bool xFixed; //This indicate if x is fixed or y is fixed

    //This part computes how angle will affect the line
    if(  0 < myAngle && myAngle < 45) //In this interval, 'y' moves faster than 'x', so we compute how many 'y's are needed to move 1 'x'
    {
        y = 1;
        x = (int) (((double) y) / (double) tan(radianAngle));
        xFixed = false;
    }

    if( 45 < myAngle && myAngle < 90)  //In this interval, 'x' moves faster than 'y', so we compute how many 'x's are needed to move 1 'y'
    {
        x = 1;
        y = (int) (((double) x) * tan(radianAngle));
        xFixed = true;
    }

    if( 90 < myAngle && myAngle < 135)  //In this interval, 'x' moves faster than 'y', so we compute how many 'x's are needed to move 1 'y'
    {
        x = 1;
        y = (int) (((double) x) * tan(radianAngle));
        xFixed = true;
    }

    if( 135 < myAngle && myAngle < 180) //In this interval, 'y' moves faster than 'x', so we compute how many 'y's are needed to move 1 'x'
    {
        y = 1;
        x = (int) (((double)  y) / tan(radianAngle));
        xFixed = false;
    }
    //EXCEPTIONAL CASES ARE TREATED HERE. ANGLES CONCERNED ARE 0, 45, 90 AND 135 DEGREES
    if(myAngle  == 0 || myAngle == 45 || myAngle == 90 || myAngle == 135)
    {
        if(myAngle == 0)
        {
            for(int i = start + 1; i < size; i++)
            {
                points[i].x = points[i - 1].x +1;
                points[i].y = points[i - 1].y;
            }
            for(int i = start - 1; i >= 0; i--)
            {
                points[i].x = points[i + 1].x - 1;
                points[i].y = points[i + 1].y;
            }

        }
        if(myAngle == 45)
        {
          for(int i = start + 1; i < size; i++)
          {
              points[i].x = points[i - 1].x + 1;
              points[i].y = points[i - 1].y + 1;
          }
          for(int i = start - 1; i >= 0; i--)
          {
              points[i].x = points[i + 1].x - 1;
              points[i].y = points[i + 1].y - 1;
          }
        }
        if(myAngle == 90)
        {
            for(int i = start + 1; i < size; i++)
            {
                points[i].x = points[i - 1].x;
                points[i].y = points[i - 1].y + 1;
            }
            for(int i = start - 1; i >= 0; i--)
            {
                points[i].x = points[i + 1].x;
                points[i].y = points[i + 1].y - 1;
            }
        }
        if(myAngle == 135)
        {
            for(int i = start + 1; i < size; i++)
            {
                points[i].x = points[i - 1].x - 1;
                points[i].y = points[i - 1].y + 1;
            }
            for(int i = start - 1; i >= 0; i--)
            {
                points[i].x = points[i + 1].x + 1;
                points[i].y = points[i + 1].y - 1;
            }
        }
    }
    else
    {
        y = abs(y);
        x = abs(x);
        int count = start + 1;

        if(xFixed)
        {
            while(count < size)
            {
              points[count].y = points[count - 1].y + 1;
              points[count].x = points[count - 1].x;

                if(count % y == 0)
                {
                    if(myAngle < 90)
                      points[count].x += 1;
                    else
                      points[count].x -= 1;
                }
                count++;
            }
        }
        else
        {
            while(count < size)
            {
                if(myAngle < 90)
                  points[count].x = points[count - 1].x + 1;
                else
                  points[count].x = points[count - 1].x - 1;

                points[count].y = points[count - 1].y;

                if(count % x == 0)
                {
                  points[count].y += 1;
                }
                count++;
            }
        }

        count = start - 1;
        if(xFixed)
        {
          while(count >= 0)
          {
            points[count].y = points[count - 1].y - 1;
            points[count].x = points[count - 1].x;

              if(count % y == 0)
              {
                  if(myAngle < 90)
                    points[count].x -= 1;
                  else
                    points[count].x += 1;
              }
              count--;
          }
        }
        else
        {
          while(count >= 0)
          {
              if(myAngle < 90)
                points[count].x = points[count - 1].x - 1;
              else
                points[count].x = points[count - 1].x + 1;

              points[count].y = points[count - 1].y;

              if(count % x == 0)
              {
                points[count].y -= 1;
              }

              count--;
          }
        }
    }
}

void Line::moveLine(int line)
{
  int xChange = mover[line].x - mover[location].x;
  int yChange = mover[line].y - mover[location].y;
  for(int i = 0; i < size; i++)
  {
      points[i].x += xChange;
      points[i].y += yChange;
  }
  location = line; //Save the new location
}
void Line::moveOneStep()
{
    moveLine(location+1);
}
