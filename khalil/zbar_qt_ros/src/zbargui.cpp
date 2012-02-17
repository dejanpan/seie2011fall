#include "zbargui.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"
#include <ros/ros.h>
#include <opencv/highgui.h>
#include "CQTImageConvertor.h"
#include <zbar_barcode_reader_node/enable_barcode_reader.h>

zbarGui::zbarGui(QWidget *parent) :
    QMainWindow(parent)
{
	std::cerr<<"Construtor called\n";
    setupUi(this);
    uvc_sub = n_.subscribe("/image_raw", 1, &zbarGui::cameraDisplay, this);
    //show();
    std::cerr<<"subscribed to topic\n";
    connect( pushButton, SIGNAL( clicked() ), this, SLOT( doStart() ));
    connect( pushButton_2, SIGNAL( clicked() ), this, SLOT( doQuit() ));

    connect(this, SIGNAL( SIG_updateImage(const IplImage*) ), this, SLOT( SLT_updateImage(const IplImage*) ) );
    connect(this, SIGNAL( SIG_updateImage2(const IplImage*) ), this, SLOT( SLT_updateImage2(const IplImage*) ) );
     client = n_.serviceClient<zbar_barcode_reader_node::enable_barcode_reader>("/barcode_reader_node/enable_barcode_reader_service");

}

zbarGui::~zbarGui()
{

}

void zbarGui::doStart()
{

	  zbar_barcode_reader_node::enable_barcode_reader srv;
	  srv.request.enable = 1;
	  std::cerr << "hello\n";
	  if (client.call(srv))
	  {
		  product_name->setText(QApplication::translate("zbarGui",srv.response.title.data.c_str() , 0, QApplication::UnicodeUTF8));
		  product_producer->setText(QApplication::translate("zbarGui",srv.response.subtitle.data.c_str() , 0, QApplication::UnicodeUTF8));
		  product_category->setText(QApplication::translate("zbarGui",srv.response.category_key.data.c_str() , 0, QApplication::UnicodeUTF8));
	  }
	  else
	  {
	    ROS_ERROR("Failed to call service ");
	    return;
	  }

	  IplImage* cv_image = NULL;
	  try
	  {
		//sensor_msgs::Image_<std::allocator<void> >*
		sensor_msgs::ImageConstPtr img_msg_ptr(new sensor_msgs::Image(srv.response.image_msg));
	  	cv_image = bridge_.imgMsgToCv(img_msg_ptr, "passthrough");
	  }
	  catch (sensor_msgs::CvBridgeException error)
	  {
	  	ROS_ERROR("error");
	  }

	  emit SIG_updateImage2(cv_image);
}


void zbarGui::doQuit()
{
    exit(0);
}

void zbarGui::cameraDisplay(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	IplImage* cv_image = NULL;
	//ROS_INFO("[BarcodeReaderNode: ] HElloooooooooo");
	try
	{
		cv_image = bridge_.imgMsgToCv(msg_ptr, "mono8");
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error");
	}

	//QImage* siImage;
	//CQTImageConvertor::showQTImage(cv_image, label);

	emit SIG_updateImage(cv_image);
}

void zbarGui::SLT_updateImage(const IplImage* pIplImage)
{
	QImage *qimg = new QImage(pIplImage->width, pIplImage->height, QImage::Format_RGB32);

	CQTImageConvertor::IplImage2QImage(pIplImage, qimg);

	label->setPixmap(QPixmap::fromImage(*qimg));

	delete qimg;

	label->show();
}

void zbarGui::SLT_updateImage2(const IplImage* pIplImage)
{
	QImage *qimg = new QImage(pIplImage->width, pIplImage->height, QImage::Format_RGB32);

	CQTImageConvertor::IplImage2QImage(pIplImage, qimg);


	label_2->setPixmap(QPixmap::fromImage(*qimg));

	delete qimg;

	label_2->show();
}
