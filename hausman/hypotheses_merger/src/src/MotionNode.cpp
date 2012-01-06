#include <observation/MotionNode.h>
#include <boost/lexical_cast.hpp>

namespace people {
MotionNode::MotionNode()
{
	dth_ = 0.2;
	weight_ = 1.0;
	node_type_ = "motion_detector";
	init();
}

MotionNode::~MotionNode()
{
}

void MotionNode::setParameter(const std::string &name, const std::string &value)
{
	if(name == "motion_weight") weight_ = boost::lexical_cast<double>(value);
}

void MotionNode::setData(const void *data, const std::string& type)
{
	if(type == "image_depth")
		img_depth_ = *(cv::Mat*)data;
	if(type == "image_motion")
		img_motion_ = *(cv::Mat*)data;
}

void MotionNode::preprocess()
{
}

double MotionNode::getConfidence(const cv::Rect &rt, double depth)
{
	double ret = 0;
	cv::Rect roi(rt.x, rt.y, rt.width, rt.height * 2.5); // face region

	float lb = depth - dth_, ub = depth + dth_;
	float stepx = (float)(roi.width-1) / 10;
	float stepy = (float)(roi.height-1) / 20;

	// already normalized
	for(float x = roi.x, i = 0; i < 10 ; x += stepx, i++) {
		for(float y = roi.y, j = 0; j < 20 ; y += stepy, j++) {
			int ix = floor(x), iy = floor(y);
			if((ix < 0) || (iy < 0) || (ix >= img_depth_.cols) || (iy >= img_depth_.rows)) continue;

			if((img_depth_.at<float>(iy, ix) > lb) && (img_depth_.at<float>(iy, ix) < ub))
			{
				ret += (float)img_motion_.at<unsigned char>(iy, ix) / 10000; // motion pixel : 100, othewise 0
			}
		}
	}

	return ret * weight_;
}

}; // Namespace
