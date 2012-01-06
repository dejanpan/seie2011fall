#include <observation/ShapeDepthNode.h>
#include <boost/lexical_cast.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream> 
#include <fstream>

namespace people {

using namespace std;

BitMaskImage::BitMaskImage():rows_(16),cols_(16)
{
	mask_.assign(rows_ * (cols_ / 8), 0);
}

BitMaskImage::BitMaskImage(size_t rows, size_t cols):rows_(rows),cols_(cols)
{
	mask_.assign(rows_ * (cols_ / 8), 0);
}

BitMaskImage::BitMaskImage(BitMaskImage &img)
{
	rows_ = img.rows_;
	cols_ = img.cols_;
	
	mask_.clear();
	for(size_t i = 0; i < img.mask_.size(); i++)
		mask_.push_back(img.mask_[i]);
}

BitMaskImage::~BitMaskImage() 
{
}
		
void BitMaskImage::setDefaultTemplate()
{
	BitMaskImage(24, 48);
	cv::Mat_<float> temp(24, 48);

	size_t offset = 0;
	temp = 0.0;

	for(size_t i = 3; i < 21; i++) {
		if (i < 6) offset = 4;
		else if (i < 9) offset = 2;
		else if (i < 12) offset = 0;
		else if (i < 15) offset = 2;
		else if (i < 18) offset = 4;
		else offset = 7;

		for(size_t j = 14 + offset; j < 34 - offset; j++) {
			temp.at<float>(i, j) = 1.0;
		}
	}

	for(size_t i = 21; i < 24; i++) {
		offset = (24 - i) * 2;
		for(size_t j = 0 + offset; j < 48 - offset; j++) {
			temp.at<float>(i, j) = 1.0;
		}
	}
	computeDepthMask(temp, 0.9f, 1.1f);
}

void BitMaskImage::computeDepthMask(cv::Mat &depth, int lb, int ub) 
{
	float row_step, col_step, r = 0, c = 0;
	int val;
	
	mask_.assign(rows_ * (cols_ / 8), 0);
	// hack.. don't wanna do time-consuming interpolation..
	// 
	row_step = (float)depth.rows / rows_;
	col_step = (float)depth.cols / cols_;

	// fprintf(stderr, "test1\n");
	for(size_t i = 0; i < rows_; i++) 
	{
		c = 0.0;
		for(size_t j = 0; j < cols_; j++) 
		{
			if(floor(r) >= depth.rows) {
				fprintf(stderr, "Error, it's gonna die.. %d > %d\n", (int)floor(r) , depth.rows);
			}
			if(floor(c) >= depth.cols) {
				fprintf(stderr, "Error, it's gonna die.. %d > %d\n", (int)floor(c) , depth.cols);
			}
			
			val = depth.at<int>((int)floor(r), (int)floor(c));
			if((val < ub) && (val > lb))
				mask_[i * (cols_/8) + (int)floor((float)j / 8)] |= 1 << (j % 8);
			c += col_step;
		}
		r += row_step;
	}
	// fprintf(stderr, "test2\n");
}

void BitMaskImage::clear() 
{
	mask_.assign(rows_ * (cols_ / 8), 0);
}

void BitMaskImage::computeDepthMask(cv::Mat &depth, float lb, float ub) 
{
	float row_step, col_step, r = 0, c = 0;
	float val;
	
	mask_.assign(rows_ * (cols_ / 8), 0);
	// hack.. don't wanna do time-consuming interpolation..
	// 
	row_step = (float)depth.rows / rows_;
	col_step = (float)depth.cols / cols_;

	for(size_t i = 0; i < rows_; i++) 
	{
		c = 0.0;
		for(size_t j = 0; j < cols_; j++) 
		{
			val = depth.at<float>((int)floor(r), (int)floor(c));
			if((val < ub) && (val > lb))
				mask_[i * (cols_/8) + (int)floor((float)j / 8)] |= 1 << (j % 8);
			c += col_step;
		}
		r += row_step;
	}
};

void BitMaskImage::getMatMask(cv::Mat &mask_mat)
{
	bool cell = false;
	for(size_t i = 0; i < rows_; i++)
	{
		for(size_t j = 0; j < cols_; j++) 
		{
			cell = ((mask_[i * (cols_/8) + (int)floor((float)j / 8)] & (1 << (j % 8))) > 0);
			mask_mat.at<unsigned char>(i, j) = 100 * cell;
			// printf("%d", cell);
		}
		// printf("\n");
	}
};

unsigned char BitMaskImage::hamdist(unsigned char x, unsigned char y)
{
	unsigned char dist = 0, val = x ^ y;
	// Count the number of set bits
	while(val)
	{
		++dist; 
		val &= val - 1;
	}
	return dist;
}

unsigned char BitMaskImage::hamdist_wocc(unsigned char x, unsigned char y, unsigned char occ)
{
	unsigned char dist = 0, val = (x ^ y) & (~occ);
	// Count the number of set bits
	while(val)
	{
		++dist; 
		val &= val - 1;
	}
	return dist;
}

unsigned int BitMaskImage::computeDistance(BitMaskImage &target, BitMaskImage &occl)
{
	unsigned int ret = 0;
	unsigned int colsize = cols_ / 8;

	for(size_t i = 0; i < rows_; i++)
		for(size_t j = 0; j < colsize; j++) 
			ret += hamdist_wocc(mask_[i * colsize + j], target.mask_[i * colsize + j], occl.mask_[i * colsize + j]);

	return ret;
};

unsigned int BitMaskImage::computeDistance(BitMaskImage &target)
{
	unsigned int ret = 0;
	unsigned int colsize = cols_ / 8;

	for(size_t i = 0; i < rows_; i++)
		for(size_t j = 0; j < colsize; j++) 
			ret += hamdist(mask_[i * colsize + j], target.mask_[i * colsize + j]);

	return ret;
}

/////////////////////////////////////
/////////////////////////////////////

ShapeDepthNode::ShapeDepthNode():ped_template_(24,48),buff_template_(24,48),empty_template_(24,48)
{
	node_type_ = "shape_depth_detector";
	dist_type_ = 0;
	hit_threshold_ = 250;
	dth_ = 0.2; // 20 cm
	weight_ = 1.0;

	init();
}

ShapeDepthNode::~ShapeDepthNode()
{
}

void ShapeDepthNode::init()
{
	ObservationNode::init();

	ped_template_.setDefaultTemplate();
}

void ShapeDepthNode::setParameter(const std::string &name, const std::string &value)
{
	if(name == "shape_depth_weight")
		weight_ = boost::lexical_cast<double>(value);
	if(name == "shape_depth_hit_threshold")
		hit_threshold_ = boost::lexical_cast<double>(value);
	if(name == "shape_depth_depth_threshold")
		dth_ = boost::lexical_cast<double>(value);
	if(name == "shape_depth_dist_type")
		dist_type_ = boost::lexical_cast<int>(value);
}

void ShapeDepthNode::setData(const void *data, const std::string& type)
{
	if(type == "image_depth")
		img_depth_ = *(cv::Mat*)data;
}

double ShapeDepthNode::getConfidence(const cv::Rect &rt, double depth)
{
	cv::Rect rt_bm = rt; rt_bm.height *= 0.6;
#if 0
	if(depth > 6.0) return 0.0;
#else
	if(depth > 7.0 || depth < 0.2) return -2.0;
#endif
	if((rt_bm.x < 0) || (rt_bm.y < 0) || (rt_bm.width <= 0) || (rt_bm.height <= 0) 
		|| (rt_bm.x + rt_bm.width > img_depth_.cols) || (rt_bm.y + rt_bm.height > img_depth_.rows))
	{
#if 1
		cv::Mat depth_roi = cv::Mat(24, 48, CV_32F);
		depth_roi = cv::Scalar(0.0);
		float rstep = (float)rt_bm.height / 24.0, cstep = (float)rt_bm.width / 48.0;
		for(float r = max(0, rt_bm.y); r < (float)min(img_depth_.rows, rt_bm.y + rt_bm.height - 1); r+=rstep) {
			for(float c = max(0, rt_bm.x); c < (float)min(img_depth_.cols, rt_bm.x + rt_bm.width - 1); c+=cstep) {

				int i = floor((r - rt_bm.y) / rstep);
				int j = floor((c - rt_bm.x) / cstep);

				depth_roi.at<float>(i, j) = img_depth_.at<float>((int)floor(r), (int)floor(c));
			}
		}
		buff_template_.computeDepthMask(depth_roi, (float)(depth - dth_), (float)(depth + dth_));
#else
		buff_template_.clear();
#endif
	}
	else{
		cv::Mat depth_roi = cv::Mat(img_depth_, rt_bm);
		CV_Assert((depth_roi.rows > 0) && (depth_roi.cols > 0));
		// compute the mask
		buff_template_.computeDepthMask(depth_roi, (float)(depth - dth_), (float)(depth + dth_));
	}
#if 0
	cv::Mat temp(24, 48, CV_8U);
	buff_template_.getMatMask(temp);
	cv::Mat temp2;
	resize(temp, temp2, cv::Size(192, 96));
	cv::imshow("bitmask", temp2);
	
	ped_template_.getMatMask(temp);
	resize(temp, temp2, cv::Size(192, 96));
	cv::imshow("template", temp2);
	cv::Mat temp_depth = img_depth_ / 4;
	cv::imshow("depth", temp_depth);
#endif
	if(dist_type_ == 1) {
		// penalize fully occupied/empty depth mask
		double conf = (double)hit_threshold_;
		conf -= (double)buff_template_.computeDistance(ped_template_);
		conf -= (double)abs((int)(400 - buff_template_.computeDistance(empty_template_)));
		conf *= (weight_ / 40);
#if 0
		// reweight by portion of non-zero depth (zero depth means no information);
		BitMaskImage temp_template(24, 48);
		temp_template.computeDepthMask(depth_roi, 0.02f, 100.0f);
		conf *= ((double)temp_template.computeDistance(empty_template_)) / (24.0 * 48.0);
#endif
		return conf;
	}
	else
	{
		return (double)((hit_threshold_ - (int)buff_template_.computeDistance(ped_template_)) / 40) * weight_;
	}
}

bool ShapeDepthNode::getShapeMat(const cv::Rect &rt, double depth, cv::Mat &shape)
{
	cv::Rect rt_bm = rt; rt_bm.height *= 0.6;

	if(depth > 7.0 || depth < 0.2) return false;
	if((rt_bm.x < 0) || (rt_bm.y < 0) || (rt_bm.width <= 0) || (rt_bm.height <= 0) 
		|| (rt_bm.x + rt_bm.width > img_depth_.cols) || (rt_bm.y + rt_bm.height > img_depth_.rows))
		return false;

	cv::Mat depth_roi = cv::Mat(img_depth_, rt_bm);
	CV_Assert((depth_roi.rows > 0) && (depth_roi.cols > 0));
	buff_template_.computeDepthMask(depth_roi, (float)(depth - dth_), (float)(depth + dth_));

	cv::Mat temp(24, 48, CV_8U);

	buff_template_.getMatMask(temp);

	shape = temp;
	return true;
}

/****************************************************
 *************** Shape Depth SVM Node ***************
 ****************************************************/

ShapeDepthSVMNode::ShapeDepthSVMNode():shape_size_(24,48)
{
	node_type_ = "shape_depth_svm_detector";
	hit_threshold_ = 0.0;
	dth_ = 0.2; // 20 cm
	weight_ = 1.0;

	init();
}

ShapeDepthSVMNode::~ShapeDepthSVMNode()
{
}

void ShapeDepthSVMNode::init()
{
	ObservationNode::init();
}

void ShapeDepthSVMNode::setParameter(const std::string &name, const std::string &value)
{
	if(name == "shape_depth_svm_weight")
		weight_ = boost::lexical_cast<double>(value);
	if(name == "shape_depth_svm_hit_threshold")
		hit_threshold_ = boost::lexical_cast<double>(value);
	if(name == "shape_depth_svm_depth_threshold")
		dth_ = boost::lexical_cast<double>(value);
	if(name == "shape_depth_svm_model_file") {
		std::cout << "load " << value << std::endl;
		assert(loadModel(value) >= 0);
	}
}

int ShapeDepthSVMNode::loadModel(const std::string &filename)
{
	std::ifstream in(filename.c_str()); 
	int cnt; std::string line, temp;

	if(!in.is_open()) { 
		cout << "Cannot open file " << filename << std::endl; 
		return -1;
	}
	model_weight_.clear();
	std::getline(in, line);
	// parse string
	cnt = 0;
	while(line.find(" ", 0) != std::string::npos) {
		size_t pos = line.find(" ", 0);
		temp = line.substr(0, pos);
		line.erase(0, pos + 1);
		model_weight_.push_back(boost::lexical_cast<double>(temp));
	}
	model_bias_ = boost::lexical_cast<double>(line);

	return 0;
}

void ShapeDepthSVMNode::setData(const void *data, const std::string& type)
{
	if(type == "image_depth")
		img_depth_ = *(cv::Mat*)data;
}

double ShapeDepthSVMNode::getConfidence(const cv::Rect &rt, double depth)
{
	cv::Rect rt_bm = rt; rt_bm.height *= 0.6;

	if(depth > 7.0 || depth < 0.2) return -2.0;

	if((rt_bm.x < 0) || (rt_bm.y < 0) || (rt_bm.width <= 0) || (rt_bm.height <= 0) 
		|| (rt_bm.x + rt_bm.width > img_depth_.cols) || (rt_bm.y + rt_bm.height > img_depth_.rows))
		return (-model_bias_ - hit_threshold_) * weight_;

	cv::Mat depth_roi = cv::Mat(img_depth_, rt_bm);
	CV_Assert((depth_roi.rows > 0) && (depth_roi.cols > 0));
	
	std::vector<unsigned char> shape;
	computeShapeVector(depth_roi, (float)(depth - dth_), (float)(depth + dth_), shape);

	double val = -model_bias_;
	for(size_t i = 0; i < shape.size(); i++) {
		if(shape[i]) {
			val += model_weight_[i];
		}
	}

	return (val - hit_threshold_) * weight_;
}

void ShapeDepthSVMNode::computeShapeVector(cv::Mat &depth, float lb, float ub, std::vector<unsigned char> &svect) 
{
	float row_step, col_step, r = 0, c = 0;
	float val;
	// hack.. don't wanna do time-consuming interpolation..
	row_step = (float)depth.rows / shape_size_.height;
	col_step = (float)depth.cols / shape_size_.width;
	svect.clear();
	// row first vectorization
	for(int j = 0; j < shape_size_.width; j++) {
		r = 0.0;
		for(int i = 0; i < shape_size_.height; i++) {
			val = depth.at<float>((int)floor(r), (int)floor(c));
			if((val < ub) && (val > lb))	svect.push_back(1);
			else							svect.push_back(0);
			r += row_step;

		}
		c += col_step;
	}
};

}; // Namespace
