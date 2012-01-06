#ifndef _MOTION_NODE_H_
#define _MOTION_NODE_H_

#include <observation/ObservationNode.h>

namespace people {
	class MotionNode : public ObservationNode 
	{
	public:
		MotionNode();
		virtual ~MotionNode();

		virtual void setParameter(const std::string &name, const std::string &value);
		virtual void setData(const void *data, const std::string &type);
		virtual double getConfidence(const cv::Rect &rt, double depth = 0);

		virtual void preprocess();
	protected:
		cv::Mat img_motion_;
		cv::Mat img_depth_;
		float dth_;

	};
}; // Namespace

#endif
