#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sstream>
#include <QtGui/QLabel>
#include "zbargui.h"
#include "ros/ros.h"
#include "ui_zbargui.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include "CQTImageConvertor.h"


#include <QApplication>

using namespace std;
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Zbar_QT");
	ros::AsyncSpinner spinner(1); // Use 4 threads
	spinner.start();
	cerr<< "11111";
	QApplication app(argc, argv);
	cerr<< "2222";
	zbarGui();
	cerr<< "333333333";
	app.exec();
	cerr<< "44444";

	return 0;
}
