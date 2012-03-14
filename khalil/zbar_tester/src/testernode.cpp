/*
 * testernode.cpp
 *
 *  Created on: Mar 2, 2012
 *      Author: banacer
 */
#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
#include "std_msgs/String.h"
#include <sstream>
#include <highgui.h>
#include <cv_bridge/cv_bridge.h>
//#include "testernode.h"
//Magick++ lib
#include <Magick++.h>
//zbar
#include <zbar.h>

using namespace zbar;
using namespace std;


class tester_node {

public:

tester_node(ros::NodeHandle &n) :
        n_(n), it_(n_)
{
  // TODO Auto-generated constructor stub
  cout << "Process started.\n";
  image_sub_ = it_.subscribe("/image_raw", 1, &tester_node::imageCallback, this);

}
void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
    try
    {
       cv_bridge_ptr_ = cv_bridge::toCvCopy(msg_ptr, "mono8");
    }
    catch (cv_bridge::Exception& e)
    {
       //ROS_ERROR("Error converting ROS Image");
    }

    // create a reader
    ImageScanner scanner;

    // configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    int width = cv_bridge_ptr_->image.cols;   // extract dimensions
    int height = cv_bridge_ptr_->image.rows;
    // obtain image data
    Magick::Blob blob(cv_bridge_ptr_->image.ptr(0), width * height);
    const void *raw = blob.data();

    // wrap image data
    Image image(width, height, "Y800", raw, width * height);

    // scan the image for barcodes
    int n = scanner.scan(image);

    // extract results
    std::stringstream ss;
    for(Image::SymbolIterator symbol = image.symbol_begin();
       symbol != image.symbol_end();
    ++symbol)
    {
        // do something useful with results
        ROS_INFO_STREAM("Publishing: barcode type: " << symbol->get_type_name()
                       << " barcode value " << symbol->get_data());
    }
    if (n == 0)
        {
            //ROS_WARN("Barcode not found");
            return;
        }

        if (n < 0)
        {
            //ROS_ERROR("Error occured while finding barcode");
            return;
        }
}

~tester_node()
{
  // TODO Auto-generated destructor stub
}

protected:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  cv_bridge::CvImagePtr cv_bridge_ptr_;

}; /* namespace std */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barcode_reader_node");
  ros::NodeHandle n("~");
  tester_node br(n);
  ros::spin();

  return 0;
}
