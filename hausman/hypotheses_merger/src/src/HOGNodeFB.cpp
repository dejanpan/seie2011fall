#include <iostream>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <observation/HOGNodeFB.h>
#include <boost/lexical_cast.hpp>

namespace people {
HOGNodeFB::HOGNodeFB() // :gpu_mutex_(boost::interprocess::open_or_create_t(), "gpu_hog")
{
	node_type_ = "hog_fb_detector";

	weight_ = 1.0;
	hit_threshold_ = 0.0;
	group_threshold_ = 2;
	det_scale_ = 1.05;

	init();
}

HOGNodeFB::~HOGNodeFB()
{
}

void HOGNodeFB::setParameter(const std::string &name, const std::string &value)
{
	if(name == "hog_weight")
		weight_ = boost::lexical_cast<double>(value);
	if(name == "hog_hit_threshold")
		hit_threshold_ = boost::lexical_cast<double>(value);
	if(name == "hog_group_threshold")
		group_threshold_ = boost::lexical_cast<int>(value);
	if(name == "hog_det_scale")
		det_scale_ = boost::lexical_cast<double>(value);
}

void HOGNodeFB::setData(const void *data, const std::string& type)
{
	if(type == "image_mono")
		img_mono_ = *(cv::Mat*)data;
}

void HOGNodeFB::quaryData(const std::string &name, void *data)
{
	if(name == "hog_fb_detection")
		*((std::vector<cv::Rect>*)data) = full_found_;
	else if(name == "hog_detection")
		*((std::vector<cv::Rect>*)data) = found_;
}

void HOGNodeFB::preprocess()
{
	std::vector<DetectionROI> location;
	DetectionROI det_roi;
	int min_x, min_y;
	int max_x, max_y;
	int stride_size = 8 ;

	preprocess_mutex_.lock();
	/////////////////////////////////////////////
	{
		found_.clear();
		confidences_.clear();
		// detect full body
		ROIHOGDetector hog_full(cv::Size(64, 128));
		std::vector<float> default_detector = cv::HOGDescriptor::getDefaultPeopleDetector();
		hog_full.setSVMDetector(default_detector);
		// HACK!! 
		// adjust aspect ratio of the bounding box to upper body compatible
		const float ub_full_ratio = 4; // (96 * hog_full.winSize.height) / (hog_full.winSize.width * 88);
		location.clear();

		double max_size = img_mono_.rows / hog_full.winSize.height; 
		//min size of ub detector
		//( hog_ub.winSize.height * ub_full_ratio ) / ( hog_full.winSize.height );
		for(double scale = 1.0; scale < max_size; scale *= det_scale_) {
			det_roi.scale = scale; // 64 by 128
			min_x = 0 - 8; min_y = 0;
			max_x = (int)floor((float)img_mono_.cols / scale) - hog_full.winSize.width + 8;
			max_y = (int)floor((float)img_mono_.rows / scale) - hog_full.winSize.height + 8;
			for(int j = min_y; j < max_y; j += 8)
				for(int i = min_x; i < max_x; i += 8)
					det_roi.locations.push_back(cv::Point(i, j));

			location.push_back(det_roi);
			det_roi.locations.clear();
		}
		hog_full.detectMultiScaleROI(img_mono_, full_found_, location, hit_threshold_, 2);
		
		// mapping the size back to upper body format
		for(size_t i = 0; i < location.size(); i++)	{
			ConfMap map;
			map.scale_ = location[i].scale;
			map.height_ = map.scale_ * hog_full.winSize.height / ub_full_ratio;

			map.one_row_size_ = floor(floor((img_mono_.cols) / location[i].scale - hog_full.winSize.width) / stride_size ) + 1;
			CV_Assert(location[i].locations[map.one_row_size_ - 1].y == location[i].locations[map.one_row_size_].y - stride_size);
			for(size_t j = 0; j < location[i].confidences.size(); j++) {
				cv::Point pt = location[i].locations[j];
				map.confidences_.push_back(location[i].confidences[j] - hit_threshold_);
				map.pts_.push_back(pt);
			}
			confidences_.push_back(map);
			// std::cout << "full scale " << map.scale_ << " height " << map.height_ << " org hieght " <<  location[i].scale * hog_full.winSize.height << std::endl;
		}

		for(size_t i = 0; i < full_found_.size(); i++) {
			cv::Rect rt = full_found_[i];
			rt.y += rt.height / 8;
			rt.height /= ub_full_ratio;
			rt.x += rt.width / 2;
			rt.width = rt.height * 96 / 88;
			rt.x -= rt.width / 2;
			found_.push_back(rt);
		}
	}
	////////////////////////////////////////////////////
	preprocess_mutex_.unlock();
}

std::vector<cv::Rect> HOGNodeFB::getDetections()
{
	return found_;
}

double HOGNodeFB::getConfidence(const cv::Rect &rt, double depth)
{
	int stride_size = 8 ;
	
	int x_pad = 15;
	int y_pad = 16;
	
	double ret = std::max(4.0 - getMinDist2Dets(found_, rt, 0.1, 0.1, 0.1), 0.0);
#if 0
	static int count = 0;
	if (count++ < 10) {
		std::cout << "rt : " 
							<< rt.x << "," 
							<< rt.y << "," 
							<< rt.width << "," 
							<< rt.height << std::endl;
		for(size_t i = 0; i < found_.size(); i++) {
			std::cout << "found : " 
								<< found_[i].x << "," 
								<< found_[i].y << "," 
								<< found_[i].width << "," 
								<< found_[i].height << std::endl;
		}
		std::cout << "min_dist : " << ret << ", " << getMinDist2Dets(found_, rt, 0.05, 0.1, 0.1) << std::endl;
	}
	else {
		assert(0);
	}
#endif
	// minimum size, no information if smaller than this!
	if(rt.height < 128 / 4)
		return ret - 10.0;

	// mapping from ub loc to fb loc!
	for(size_t i = 0; i < confidences_.size(); i++) {
		// size check
		if(rt.height >= (confidences_[i].height_ * (1 + 1 / det_scale_) / 2) 
			&& rt.height <= (confidences_[i].height_ * (1 + det_scale_) / 2)) {
			// get pointer to the confidence value
			int xidx = floor((float)((rt.x / confidences_[i].scale_) - x_pad + stride_size / 2) / stride_size);
			int yidx = floor((float)((rt.y / confidences_[i].scale_) - y_pad + stride_size / 2) / stride_size);

			if((xidx < 0) || (yidx < 0) || (xidx >= confidences_[i].one_row_size_)) break;
			int ptidx = yidx * confidences_[i].one_row_size_ + xidx;
			if(ptidx >= (int)confidences_[i].confidences_.size()) break;

			CV_Assert(abs((rt.x / confidences_[i].scale_ - x_pad )- confidences_[i].pts_[ptidx].x) <= 4);
			if(confidences_[i].scale_ >= 1.0) {
				CV_Assert(abs((rt.y / confidences_[i].scale_ - y_pad ) - confidences_[i].pts_[ptidx].y) <= 4);
			}

			return (ret + confidences_[i].confidences_[ptidx]) * weight_;
		}
	}

	return 0; // obs_out_of_image;
}

}; // Namespace
