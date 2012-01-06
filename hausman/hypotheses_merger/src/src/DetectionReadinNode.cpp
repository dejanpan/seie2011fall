#include <iostream>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <observation/DetectionReadinNode.h>
#include <boost/lexical_cast.hpp>
#include <common/util.h>

namespace people {
DetectionReadinNode::DetectionReadinNode()
{
	node_type_ = "detection_readin_node";

	time_sec_ = 0.0;
	weight_ = 1.0;
	hit_threshold_ = 0.0;
	init();
}

DetectionReadinNode::~DetectionReadinNode()
{
}

void DetectionReadinNode::setParameter(const std::string &name, const std::string &value)
{
	if(name == "detection_readin_weight")
		weight_ = boost::lexical_cast<double>(value);
	if(name == "detection_readin_prefix")
		prefix_ = value;
	if(name == "detection_readin_det_scale")
		det_scale_ = boost::lexical_cast<double>(value);
	if(name == "detection_readin_threshold")
		hit_threshold_ = boost::lexical_cast<double>(value);
}

void DetectionReadinNode::setData(const void *data, const std::string& type)
{
	if(type == "image_mono")
	{
		cv::Mat image = *(cv::Mat*)data;
		imsize_.width = image.cols;
		imsize_.height = image.rows;
	}
	if(type == "time_sec")
		time_sec_ = *(double*)data;
}

void DetectionReadinNode::quaryData(const std::string &name, void *data)
{
	if(name == "detection_readin_detection")
		*((std::vector<cv::Rect>*)data) = found_;
}

void DetectionReadinNode::preprocess()
{
	ostringstream filename;

	int idx = floor((time_sec_ - floor(time_sec_ / 10000) * 10000) * 100);
	filename << prefix_ << idx << ".conf";

	std::cout << "read conf file " << filename.str() << std::endl;

	assert(readDetectionResult(filename.str()));
}

void DetectionReadinNode::dbgShowConfImage(DetectionReadinConfidence &conf)
{
	cv::Mat confidence_image(imsize_.height, imsize_.width, CV_8U);
	confidence_image = cv::Scalar(obs_out_of_image);

	float minval = 100000000.0f;
	float maxval = -10000000.0f;
#if 0
	for(int i = 0; i < conf.map_.rows; i++) {
		for(int j = 0; j < conf.map_.cols; j++) {
			if(minval > conf.map_.at<float>(i, j)) {
				minval = conf.map_.at<float>(i, j);
			}
			if(maxval < conf.map_.at<float>(i, j)) {
				maxval = conf.map_.at<float>(i, j);
			}
		}
	}
#endif
	minval = -7;
	maxval = 1;

	float scale = 255 / (maxval - minval);

	for(int i = 0; i < conf.map_.rows; i++) {
		for(int j = 0; j < conf.map_.cols; j++) {
			unsigned char value = floor((conf.map_.at<float>(i, j) - minval) * scale);
			int half_step = ceil(conf.step_ / 2);

			for(int di = -half_step; di < half_step; di++) {
				for(int dj = -half_step; dj < half_step; dj++) {
					int x = conf.minx_ + dj + floor(conf.step_ * j);
					int y = conf.miny_ + di + floor(conf.step_ * i);

					if((x >= 0) && (y >= 0) && (x < 640) && (y < 480))
						confidence_image.at<unsigned char>(y, x) = value;
				}
			}
		}
	}
	std::cout << std::endl;
	std::cout << "size : " << conf.size_ << std::endl;
	std::cout << "size_ratio : " << conf.size_ratio_ << std::endl;
	std::cout << "minx : " << conf.minx_ << std::endl;
	std::cout << "miny : " << conf.miny_ << std::endl;
	std::cout << "step : " << conf.step_ << std::endl;
	std::cout << "map size : [width " << conf.map_.cols << ", height " << conf.map_.rows << "]" << std::endl;

	cv::imshow("confidence", confidence_image);
	cv::waitKey();
}

bool DetectionReadinNode::readDetectionResult(const std::string filename)
{
	FILE *fp;
	size_t nread;
	fp = fopen(filename.c_str(), "r");
	if(fp == NULL) {
		std::cout << "ERROR :Cannot read detection confidence file!" << std::endl;
		fclose(fp);
		return false;
	}
	found_.clear();
	confidences_.clear();
	
	char header[4];
	nread = fread(header, sizeof(char), 4, fp);
	assert(nread == 4);
	if(!(header[0] == 'C'	&& header[1] == 'O'
		&& header[2] == 'N'	&& header[3] == 'F')) {
		std::cout << "ERROR : invalid header format!" << std::endl;
		fclose(fp);
		return false;
	}

	unsigned int nums;
	float det[5];

	assert(sizeof(unsigned int) == 4);

	nread = fread(&nums, sizeof(unsigned int), 1, fp);
	assert(nread == 1);
	for(size_t i = 0; i < nums; i++) {
		nread = fread(det, sizeof(float), 5, fp);
		assert(nread == 5);

		cv::Rect rt(det[0], det[1], det[2] - det[0] + 1, det[3] - det[1] + 1); // LSVM format (l, t, r, b) -> (l, t, w, h)
		found_.push_back(rt);
	}

	nread = fread(&nums, sizeof(unsigned int), 1, fp);
	assert(nread == 1);
	// float prev_size = 0.0;
	for(size_t i = 0; i < nums; i++) {
		DetectionReadinConfidence conf;
		float *data;
		float map_size[2];	int imap_size[2];

		nread = fread(&conf.size_, sizeof(float), 1, fp);
		assert(nread == 1);
		nread = fread(&conf.size_ratio_, sizeof(float), 1, fp);
		assert(nread == 1);
		nread = fread(&conf.step_, sizeof(float), 1, fp);
		assert(nread == 1);
		nread = fread(&conf.minx_, sizeof(float), 1, fp);
		assert(nread == 1);
		nread = fread(&conf.miny_, sizeof(float), 1, fp);
		assert(nread == 1);
		nread = fread(map_size, sizeof(float), 2, fp); // my mistake....should have used integer....
		assert(nread == 2);

		imap_size[0] = round(map_size[0]);
		imap_size[1] = round(map_size[1]);

		data = new float [ imap_size[0] * imap_size[1] ];
		nread = fread(data, sizeof(float), imap_size[0] * imap_size[1], fp);
		
		conf.map_ = cv::Mat(imap_size[0], imap_size[1], CV_32F);
		for(int row = 0; row < imap_size[0]; row++) {
			for(int col = 0; col < imap_size[1]; col++) {
#if 0
				conf.map_.at<float>(row, col) = data[ row + col * imap_size[0] ] - hit_threshold_;
#else
				float temp = data[ row + col * imap_size[0] ];
				// std::cout << "row : " << temp << std::endl;
				temp = (- log(0.7378) - pow((temp + 1.6036) / 0.7378, 2) / 2)  
					 - (- log(0.6232) - pow((temp + 2.9418) / 0.6232, 2) / 2);
				// std::cout << "lkhood : " << temp << std::endl;
				// assert(temp < 0);
				conf.map_.at<float>(row, col) = temp;
#endif
			}
		}
		delete data;

		confidences_.push_back(conf);
	}
	fclose(fp);

	return true;
}

std::vector<cv::Rect> DetectionReadinNode::getDetections()
{
	return found_;
}

double DetectionReadinNode::detectionOberlap(const cv::Rect &rt)
{
	double ret = 0;
	double ol = 0;

	// find the maximum overlap
	for(size_t i = 0; i < found_.size(); i ++ ) {
		ol = bb_overlap(rt, found_[i]);
		if(ret < ol) ret = ol;
	}
	assert(ret >= 0); assert(ret <= 1);
	return ret * 3.0f;
}

double DetectionReadinNode::getConfidence(const cv::Rect &rt, double depth)
{
	double overlap = 0.0; // detectionOberlap(rt);

	cv::Point pt(rt.x + rt.width / 2, rt.y + rt.height / 2);

	// minimum size
	if(rt.height < 64)
		return 0;

	// printf("confidences count %d : searching for [%d,%d,%d,%d]\n", confidences_.size(),rt.x, rt.y, rt.width, rt.height);
	for(size_t i = 0; i < confidences_.size(); i++) {
		// printf("confidence%d height %d\n", i, (int)confidences_[i].size_);
		if(rt.height >= (confidences_[i].size_ * (1 + 1 / det_scale_) / 2) 
			&& rt.height <= (confidences_[i].size_ * (1 + det_scale_) / 2)) {
			int x = floor((pt.x - confidences_[i].minx_) / confidences_[i].step_);
			int y = floor((pt.y - confidences_[i].miny_) / confidences_[i].step_);
			// check whether point is in image
			if((x < 0) || (y < 0) || (x > confidences_[i].map_.cols) || (y > confidences_[i].map_.rows)) {
				return (overlap + obs_out_of_image) * weight_;
			}
			// std::cout << "[" << x << ", " << y << "] : value " << confidences_[i].map_.at<float>(y, x) << " " << confidences_[i].map_.at<float>(y, x) * weight_ << std::endl;
			// dbgShowConfImage(confidences_[i]);
			return (overlap + confidences_[i].map_.at<float>(y, x)) * weight_;
		}
	}

	return (overlap + obs_out_of_image) * weight_;
}
}; // Namespace
