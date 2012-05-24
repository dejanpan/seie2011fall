/*
 * Line.h
 *
 *  Created on: Mar 22, 2012
 *      Author: banacer
 */

#ifndef LINE_H_
#define LINE_H_
#include "opencv2/core/core.hpp"

class Line
{
private:
  int* values;
  struct myPoint* points;
  int size;
  int angle;
  bool updated;
  struct myPoint* mover; //This line is perpendicular to the line in question. The line in question move on this one.
  int location; //This indicates where the line is on the mover. This value represents the subscript of the mover.
  int centerPoint; // The index of the line to be decided the starting point meaning have at the beginning (0,0) or (ymax,0)
public:
  int* getValues();
  int getValue(int );
  void setValue(int, int );
  struct myPoint getPoint(int );
  void setPoint(struct myPoint ,int );
  int getAngle();
  void setAngle(int );
  int getCenterPoint();
  void setCenterPoint(int );
  int stepMove();
  int getSize();
  bool isUpdated();
  void update(cv::Mat );
  void chooseStartingPoint(cv::Mat );
  bool graspsImage(cv::Mat img);
  bool pointInImage(cv::Mat );
  void constructMover();
  void constructLine(int ,int );
  void moveLine(int );
  void moveOneStep();
  Line();
  Line(int size,int angle);
  virtual ~Line();
};

struct myPoint{
  int x;
  int y;
};

#endif /* LINE_H_ */
