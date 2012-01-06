#ifndef _HOG_NODE_FB_H_
#define _HOG_NODE_FB_H_

#include <observation/ObservationNode.h>
#include <observation/roihog.h>

namespace people {
	class HOGNodeFB : public ObservationNode 
	{
	public:
		HOGNodeFB();
		virtual ~HOGNodeFB();

		virtual void setParameter(const std::string &name, const std::string &value);
		virtual void setData(const void *data, const std::string &type);
		virtual double getConfidence(const cv::Rect &rt, double depth = 0);

		virtual void preprocess();
		virtual std::vector<cv::Rect> getDetections();

		void quaryData(const std::string &name, void *data);
	protected:
		cv::Mat img_mono_;
		std::vector<cv::Rect> found_;
		// std::vector<cv::Rect> ub_found_;
		std::vector<cv::Rect> full_found_;
		std::vector<ConfMap> confidences_;
		// parameters
		double hit_threshold_;
		int group_threshold_;
		double det_scale_;
	};
}; // Namespace
#endif
