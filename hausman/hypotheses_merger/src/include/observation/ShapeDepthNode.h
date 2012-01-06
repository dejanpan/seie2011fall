#ifndef _SHAPE_DEPTH_NODE_H_
#define _SHAPE_DEPTH_NODE_H_

#include <observation/ObservationNode.h>

namespace people {
	class BitMaskImage
	{
	public:
		BitMaskImage();
		BitMaskImage(size_t rows, size_t cols);
		BitMaskImage(BitMaskImage &img);

		virtual ~BitMaskImage() ;
		
		void clear();
		void setDefaultTemplate();

		void computeDepthMask(cv::Mat &depth, int lb, int ub);
		void computeDepthMask(cv::Mat &depth, float lb, float ub);
		void getMatMask(cv::Mat &mask_mat);

		unsigned char hamdist(unsigned char x, unsigned char y);
		unsigned char hamdist_wocc(unsigned char x, unsigned char y, unsigned char occ);
		unsigned int computeDistance(BitMaskImage &target, BitMaskImage &occl);
		unsigned int computeDistance(BitMaskImage &target);
	protected:
		std::vector<unsigned char>mask_;
		size_t	rows_;
		size_t	cols_;
	};

	class ShapeDepthNode : public ObservationNode 
	{
	public:
		ShapeDepthNode();
		virtual ~ShapeDepthNode();

		virtual void setParameter(const std::string &name, const std::string &value);
		virtual void setData(const void *data, const std::string &type);
		virtual double getConfidence(const cv::Rect &rt, double depth = 0);

		virtual bool getShapeMat(const cv::Rect &rt, double depth, cv::Mat &shape);

		virtual void init();
	protected:
		cv::Mat img_depth_;

		BitMaskImage ped_template_;
		BitMaskImage buff_template_;
		BitMaskImage empty_template_;

		double hit_threshold_;
		double dth_;

		int dist_type_;
	};

	class ShapeDepthSVMNode : public ObservationNode
	{
	public:
		ShapeDepthSVMNode();
		virtual ~ShapeDepthSVMNode();

		virtual void 	init();

		virtual void 	setParameter(const std::string &name, const std::string &value);
		virtual void 	setData(const void *data, const std::string &type);
		virtual double 	getConfidence(const cv::Rect &rt, double depth = 0);
	private:
		// get vector of the shape
		void 	computeShapeVector(cv::Mat &depth, float lb, float ub, std::vector<unsigned char> &svect);
		// load SVM model
		int 	loadModel(const std::string &filename);
	protected:
		// depth image storage
		cv::Mat 			img_depth_;
		// shape vector parameters
		cv::Size			shape_size_;
		double 				dth_;
		// model parameters
		std::vector<double>	model_weight_;
		double				model_bias_;
		// additional parameter
		double				hit_threshold_;
	};
}; // Namespace

#endif
