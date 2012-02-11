#include "zbargui.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"
#include <ros/ros.h>
#include <opencv/highgui.h>
#include "CQTImageConvertor.h"

zbarGui::zbarGui(QWidget *parent) :
    QMainWindow(parent)
{
	std::cerr<<"8888888";
    setupUi(this);
    //uvc_sub = n_.subscribe("/image_raw", 1, &zbarGui::cameraDisplay, this);
    show();
    std::cerr<<"8888888";
    connect( pushButton, SIGNAL( clicked() ), this, SLOT( doStart() ));
    connect( pushButton_2, SIGNAL( clicked() ), this, SLOT( doQuit() ));
}

zbarGui::~zbarGui()
{

}

void zbarGui::doStart()
{
	//sensor_msgs::CvBridge bridge_;
	sensor_msgs::ImageConstPtr imgmasg_ptr = ros::topic::waitForMessage<sensor_msgs::Image>("/image_raw", ros::Duration(5.0));
	IplImage* cv_image = NULL;


		ROS_INFO("[BarcodeReaderNode: ] HElloooooooooo");

		try
		{
			cv_image = bridge_.imgMsgToCv(imgmasg_ptr, "mono8");
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("error");
		}
		QImage* siImage;
		CQTImageConvertor::showQTImage(cv_image, label);
		label->show();
}


void zbarGui::doQuit()
{
    exit(0);
}

void zbarGui::cameraDisplay(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	//ROS_INFO("[BarcodeReaderNode: ] HElloooooooooo");
	IplImage* cv_image = NULL;
	ROS_INFO("[BarcodeReaderNode: ] HElloooooooooo");
	try
	{
		cv_image = bridge_.imgMsgToCv(msg_ptr, "mono8");
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error");
	}

	//QImage* siImage;
	CQTImageConvertor::showQTImage(cv_image, label);
	label->show();
}
