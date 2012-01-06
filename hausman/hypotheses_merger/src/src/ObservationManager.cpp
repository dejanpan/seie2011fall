#include <stdio.h>
#ifdef HAVE_TBB
#include <tbb/tbb.h>
#include <tbb/tbb_stddef.h>
#endif
#include <iostream>
#include <observation/ObservationManager.h>
#include <common/util.h>

namespace people {
#define UB_HEIGHT 0.6
#ifdef HAVE_TBB
	struct PreprocessInvoker
	{
			PreprocessInvoker(std::vector<ObservationNode*> nodes) 
			{
				nodes_ = nodes;
			}
			
			void operator()( const tbb::blocked_range<int>& range ) const
			{
					int i, i1 = range.begin(), i2 = range.end();
					for( i = i1; i < i2; i++ )
						nodes_[i]->preprocess();
			}

			std::vector<ObservationNode*> nodes_;
	};
#endif
	ObservationManager::ObservationManager()
	{
		min_height_ = 1.1;
		max_height_ = 2.3;

		obs_lkhood_out_of_height_ = -15.0; // heavily penalize too tall/small human
	}

	ObservationManager::~ObservationManager()
	{
		releaseNodes();
	}

	void ObservationManager::releaseNodes()
	{
		for(size_t i = 0; i < nodes_.size(); i++)
			delete nodes_[i];
		nodes_.clear();
	}

	void ObservationManager::setData(const void *data, const std::string &type)
	{
		// need the depth image to get 3d point from rect.
		if(type == "image_depth") img_depth_ = *(cv::Mat*)data;
		if(type == "image_color") img_color_ = *(cv::Mat*)data;
		if(type == "time_sec") { 
			time_sec_ = *(double*)data;
			std::cout << "data recved " << time_sec_ - floor(time_sec_/1000) * 1000 << std::endl;
		}

		std::vector<ObservationNode*>::iterator it;
		for(it = nodes_.begin(); it < nodes_.end(); it++)
			(*it)->setData(data, type);
	}

	void ObservationManager::setParameters(const std::string &name, const std::string &value)
	{
		if(name == "min_height") min_height_ = boost::lexical_cast<double>(value);
		if(name == "max_height") max_height_ = boost::lexical_cast<double>(value);
		if(name == "total_weight") 
			total_weight_ = boost::lexical_cast<double>(value);

		std::vector<ObservationNode*>::iterator it;
		for(it = nodes_.begin(); it < nodes_.end(); it++)
			(*it)->setParameter(name, value);
	}

	ObservationNode* ObservationManager::getObservationNode(std::string &type)
	{
		std::vector<ObservationNode*>::iterator it;
		for(it = nodes_.begin(); it < nodes_.end(); it++)
			if(type == (*it)->getType()) return *it;
		return NULL;
	}

	void ObservationManager::insertObservationNode(ObservationNode* node)
	{
		nodes_.push_back(node);
	}

	void ObservationManager::setCameraModel (image_geometry::PinholeCameraModel &cam_model, btMatrix3x3 ref_to_cam_rot, btVector3 ref_to_cam_trans
																																												, btMatrix3x3 cam_to_ref_rot, btVector3 cam_to_ref_trans)
	{
		cam_model_ = cam_model;

		ref_to_cam_rot_ = ref_to_cam_rot;
		ref_to_cam_trans_ = ref_to_cam_trans;

		cam_to_ref_rot_ = cam_to_ref_rot;
		cam_to_ref_trans_ = cam_to_ref_trans;
	}

	PeopleStatePtr ObservationManager::getStateFromRect(cv::Rect &rt)
	{
		assert(img_depth_.rows > 0);
		assert(img_depth_.cols > 0);
		
		cv::Point im_top(rt.x + rt.width / 2, rt.y);
		btVector3 pt3;
		// get median depth of the bounding box.
		cv::Rect roi = cv::Rect(rt.x + 0.3 * rt.width, rt.y + 0.1 * rt.height, rt.width * 0.4, rt.height * 0.7);
		// TODO : may need to ensure roi is inside of the image
		if((roi.x < 0) ||  (roi.y < 0) 
			|| (roi.x + roi.width >= img_depth_.cols) || (roi.y + roi.height >= img_depth_.rows)
			|| (roi.width <= 0) || (roi.height <= 0))
		{
			PeopleStatePtr state = boost::make_shared<PeopleState>(PeopleState());
			state->x_ = 0;		state->y_ = 0;		state->z_ = 0;
			return state;
		}

		// cv::Mat depth_roi_temp(img_depth_, roi);
		cv::Mat tmat(1, roi.width * roi.height, CV_32FC1);
		// = depth_roi_temp.reshape(1, depth_roi_temp.rows * depth_roi_temp.cols);
		
		int cnt = 0;
		for(int i = 0; i < roi.width; i++) {
			for(int j = 0; j < roi.height; j++, cnt++)
				tmat.at<float>(0, cnt) = img_depth_.at<float>(roi.y + j, roi.x + i);
		}

		cv::Mat tmat_sorted;
		cv::sort(tmat, tmat_sorted, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING);
		// in camera coordinate, depth is z!
		pt3[ 2 ] = tmat_sorted.at<float>(floor(cv::countNonZero(tmat_sorted) / 2.0));
		if((pt3[ 2 ] == 0) || (pt3[ 2 ] > 15))
		{
			// need to find alternative way to get the proposals.
			PeopleStatePtr state = boost::make_shared<PeopleState>(PeopleState());
#if 1
			cv::Point im_bottom(rt.x + rt.width / 2, rt.y + rt.height);

			cv::Point3d ray1 = cam_model_.projectPixelTo3dRay(im_top);
			cv::Point3d ray2 = cam_model_.projectPixelTo3dRay(im_bottom);
			assert(ray1.z == ray2.z);

			pt3[ 2 ] = UB_HEIGHT * ray1.z / (ray2.y - ray1.y);

			pt3[ 0 ] = pt3[ 2 ] * ray1.x / ray1.z;
			pt3[ 1 ] = pt3[ 2 ] * ray1.y / ray1.z;

			pt3 = cam_to_ref_rot_ * pt3 + cam_to_ref_trans_;
			state->x_ = pt3[ 0 ];		state->y_ = pt3[ 1 ];		state->z_ = pt3[ 2 ];
#else
			state->x_ = 0;		state->y_ = 0;		state->z_ = 0;
#endif
			return state;
		}
		// get inverse-projection
		cv::Point3d ray = cam_model_.projectPixelTo3dRay(im_top);
		pt3[ 0 ] = pt3[ 2 ] * ray.x / ray.z;
		pt3[ 1 ] = pt3[ 2 ] * ray.y / ray.z;
		// transform to reference co-ordinate
		pt3 = cam_to_ref_rot_ * pt3 + cam_to_ref_trans_;

		PeopleStatePtr state = boost::make_shared<PeopleState>(PeopleState());
		state->x_ = pt3[ 0 ];		state->y_ = pt3[ 1 ];		state->z_ = pt3[ 2 ];
		return state;
	}

	double ObservationManager::getDepthFromState(PeopleStatePtr state)
	{
		btVector3 pt3;
		pt3[ 0 ] = state->x_; 	pt3[ 1 ] = state->y_;		pt3[ 2 ] = state->z_;
		// get point in camera co-ordinate
		pt3 = ref_to_cam_rot_ * pt3 + ref_to_cam_trans_;
		return pt3[ 2 ];
	}

	cv::Rect ObservationManager::getRectFromState(PeopleStatePtr state)
	{
		btVector3 pt3;
		pt3[ 0 ] = state->x_; 	pt3[ 1 ] = state->y_;		pt3[ 2 ] = state->z_;
		// get point in camera co-ordinate
		pt3 = ref_to_cam_rot_ * pt3 + ref_to_cam_trans_;

		cv::Rect rt;
		cv::Point2d uv_feet;	cv::Point2d uv_head;
		cv::Point3d pt(pt3[ 0 ], pt3[ 1 ], pt3[ 2 ]);

		uv_head = cam_model_.project3dToPixel(pt);
		pt.y += UB_HEIGHT; 
		uv_feet = cam_model_.project3dToPixel(pt);

		rt.height = (uv_feet.y - uv_head.y);
		rt.width = rt.height;
		rt.x = uv_head.x - rt.width / 2;
		rt.y = uv_head.y;

		return rt;
	}

	void ObservationManager::preprocess()
	{
#ifdef HAVE_TBB
		tbb::parallel_for(tbb::blocked_range<int>(0, nodes_.size()), PreprocessInvoker(nodes_));
#else
		// may need to consider running in multiple threads
		std::vector<ObservationNode*>::iterator it;
		for(it = nodes_.begin(); it < nodes_.end(); it++) {
			(*it)->preprocess();
		}
#endif
	}

	class CV_EXPORTS SimilarProposals
	{
	public:    
		SimilarProposals(double delta) : delta_sq_(delta * delta) {}
		inline bool operator()(const PeopleStatePtr p1, const PeopleStatePtr p2) const
		{
			return pow(p1->x_ - p2->x_, 2) + pow(p1->y_ - p2->y_, 2) + pow(p1->z_ - p2->z_, 2) < delta_sq_;
		}    
		double delta_sq_; 
	};   

	std::vector<PeopleStatePtr> ObservationManager::getDetections2(const std::vector<PeopleStatePtr> &proposals)
	{
		std::vector<cv::Rect> rts;
		std::vector<PeopleStatePtr> dets;

		// get detections
		rts = getBoundingBoxes(rts, 0.0f);

		// get 3D location
		for(size_t i = 0; i < rts.size(); i++)
		{
			PeopleStatePtr temp = getStateFromRect(rts[i]);
			if((temp->z_ > min_height_) && (temp->z_ < max_height_)) {
				dets.push_back(temp);
			}
		}
		for(size_t i = 0; i < proposals.size(); i++)
			dets.push_back(proposals[i]);

		// group all
		std::vector<int> labels;
		cv::partition(dets, labels, SimilarProposals(0.2));
		
		// get maximum confidence points in each group
		std::vector<PeopleStatePtr> rets;
		for(size_t i = 0; i < labels.size(); i++) {
			int maxidx = -1;
			double maxconf = -10.0f; // threshold for proposals
			for(size_t j = 0; j < labels.size(); j++) {
				if(labels[j] == (int)i)
				{
					double conf = getConfidence(dets[j]);
					if(conf > maxconf) {
						maxidx = j;
						maxconf = conf;
					}
				}
			}
			if(maxidx >= 0) {
				rets.push_back(dets[maxidx]);
			}
		}

		return rets;
	}

	std::vector<PeopleStatePtr> ObservationManager::getDetections(const std::vector<PeopleStatePtr> &proposals)
	{
		std::vector<cv::Rect> rts;
		
		for(size_t i = 0; i < proposals.size(); i++)
		{
			cv::Rect rt = getRectFromState(proposals[i]);
			rts.push_back(rt);
			// trick for groupRectangles..
			rts.push_back(rt);
		}

		rts = getBoundingBoxes(rts);

		std::vector<PeopleStatePtr> ret;
		for(size_t i = 0; i < rts.size(); i++)
		{
			PeopleStatePtr temp = getStateFromRect(rts[i]);
			if((temp->z_ > min_height_) && (temp->z_ < max_height_)) {
				ret.push_back(temp);
			}
		}
#if 0
		rts.clear();
		for(size_t i = 0; i < ret.size(); i++)
		{
			cv::Rect rt = getRectFromState(ret[i]);
			rts.push_back(rt);
			// trick for groupRectangles..
			rts.push_back(rt);
		}
		rts = getBoundingBoxes(rts);

		ret.clear();
		for(size_t i = 0; i < rts.size(); i++)
		{
			PeopleStatePtr temp = getStateFromRect(rts[i]);
			if((temp->z_ > 1.0) && (temp->z_ < 2.3)) {
				ret.push_back(temp);
			}
		}
#endif
		return ret;
	}

	std::vector<cv::Rect> ObservationManager::getBoundingBoxes(const std::vector<cv::Rect> &proposals, double eps)
	{
		std::vector<cv::Rect> ret = proposals;
		std::vector<cv::Rect> temp;
		// get detections and return them
		// if there are overlapping detections, return only one of them.
		std::vector<ObservationNode*>::iterator it;
		for(it = nodes_.begin(); it < nodes_.end(); it++)
		{
			temp = (*it)->getDetections();
#if 1
			for(size_t i = 0; i < temp.size(); i++) {
				ret.push_back(temp[i]);
				// trick..
				ret.push_back(temp[i]);
			}
#else
			ret.insert(ret.end(), temp.begin(), temp.end());
			// trick..
			ret.insert(ret.end(), temp.begin(), temp.end());
#endif
		}
		cv::groupRectangles(ret, 1, eps);
		return ret;
	}

	double ObservationManager::getConfidence(PeopleStatePtr state, std::string type)
	{
		double ret = 0;
		cv::Rect rt;
		double depth = state->z_;

		// assuming z is height direction!!!
		if((state->z_ < min_height_) || (state->z_ > max_height_))
			return obs_lkhood_out_of_height_;

		// get image projection.
		rt = getRectFromState(state);

		btVector3 pt3;
		pt3[ 0 ] = state->x_; 	pt3[ 1 ] = state->y_;		pt3[ 2 ] = state->z_;
		// get point in camera co-ordinate
		pt3 = ref_to_cam_rot_ * pt3 + ref_to_cam_trans_;
		depth = pt3 [ 2 ];

		if(depth < 0.5) { // not possible to detect anyway..
			return obs_lkhood_out_of_height_;
		}

		if(type == std::string("all")) {
			// iterate over all observation nodes_
			std::vector<ObservationNode*>::iterator it;
			for(it = nodes_.begin(); it < nodes_.end(); it++) 
#if 1
				ret += soft_max((*it)->getConfidence(rt, depth), 4.0f);
#else
				ret += (*it)->getConfidence(rt, depth);
#endif
		}
		else {
			std::vector<ObservationNode*>::iterator it;
			for(it = nodes_.begin(); it < nodes_.end(); it++)
			{
				if((*it)->getType() == type)
#if 1
					ret += soft_max((*it)->getConfidence(rt, depth), 4.0f);
#else
					ret += (*it)->getConfidence(rt, depth);
#endif
			}
		}

		ret *= total_weight_;

		return ret;
	}

	void ObservationManager::quaryData(const std::string &name, void *data)
	{
		std::vector<ObservationNode*>::iterator it;
		for(it = nodes_.begin(); it < nodes_.end(); it++)
		{
			(*it)->quaryData(name, data);
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	// Observation Manager for Camera Estimate Top Hierarchy Class
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	// Observation Manager for General Camera Model
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	ObservationManagerGeneralCam::ObservationManagerGeneralCam()
	{
		feat_tracker_ = NULL;
		ObservationManager::ObservationManager();
	}

	ObservationManagerGeneralCam::~ObservationManagerGeneralCam()
	{
		releaseNodes();
	}

	void ObservationManagerGeneralCam::setCameraModel(image_geometry::PinholeCameraModel &cam_model)
	{
		cam_model_ = cam_model;
	}

	void ObservationManagerGeneralCam::setParameters(const std::string &name, const std::string &value)
	{
		if(name == "feat_sigma_u")  gfeat_sigma_u_ = boost::lexical_cast<double>(value);
		else if(name == "feat_sigma_v")  gfeat_sigma_v_ = boost::lexical_cast<double>(value);
		else if(name == "feat_sigma_d")  gfeat_sigma_d_ = boost::lexical_cast<double>(value);

		ObservationManager::setParameters(name, value);
	}

	cv::Rect ObservationManagerGeneralCam::getRectFromState(PeopleStatePtr ped_state, CamStatePtr cam_state)
	{
		btVector3 pt3;
		btMatrix3x3 cam_rot = cam_state->getRot();
		btVector3 cam_trans = cam_state->getTrans();

		pt3[ 0 ] = ped_state->x_; pt3[ 1 ] = ped_state->y_; pt3[ 2 ] = ped_state->z_;
		// get point in camera co-ordinate
		pt3 = cam_rot.inverse() * (pt3 - cam_trans);

		cv::Rect rt;
		cv::Point2d uv_feet;	cv::Point2d uv_head;
		cv::Point3d pt(pt3[ 0 ], pt3[ 1 ], pt3[ 2 ]);

		uv_head = cam_model_.project3dToPixel(pt);
		pt.y += UB_HEIGHT; 
		uv_feet = cam_model_.project3dToPixel(pt);

		rt.height = (uv_feet.y - uv_head.y);
		rt.width = rt.height;
		rt.x = uv_head.x - rt.width / 2;
		rt.y = uv_head.y;

		return rt;
	}

	cv::Point3f ObservationManagerGeneralCam::getPointFromState(GFeatStatePtr feat_state, CamStatePtr cam_state)
	{
		btVector3 pt3;
		btMatrix3x3 cam_rot = cam_state->getRot();
		btVector3 cam_trans = cam_state->getTrans();

		pt3[ 0 ] = feat_state->x_; pt3[ 1 ] = feat_state->y_; pt3[ 2 ] = feat_state->z_;
		// get point in camera co-ordinate
		pt3 = cam_rot.inverse() * (pt3 - cam_trans);

		cv::Point2d uv;
		cv::Point3d pt(pt3[ 0 ], pt3[ 1 ], pt3[ 2 ]);
		uv = cam_model_.project3dToPixel(pt);
		
		// return (x, y, depth)
		return cv::Point3f(uv.x, uv.y, pt3[ 2 ]);
	}

	PeopleStatePtr ObservationManagerGeneralCam::getPeopleStateFromRect(cv::Rect &rt, CamStatePtr cam_state)
	{
		assert(img_depth_.rows > 0);
		assert(img_depth_.cols > 0);
		
		cv::Point im_top(rt.x + rt.width / 2, rt.y);
		btVector3 pt3;
		// get median depth of the bounding box.
		cv::Rect roi = cv::Rect(rt.x + 0.3 * rt.width, rt.y + 0.1 * rt.height, rt.width * 0.4, rt.height * 0.7);
		// TODO : may need to ensure roi is inside of the image
		if((roi.x < 0) ||  (roi.y < 0) 
			|| (roi.x + roi.width >= img_depth_.cols) || (roi.y + roi.height >= img_depth_.rows)
			|| (roi.width <= 0) || (roi.height <= 0))
		{
			PeopleStatePtr state = boost::make_shared<PeopleState>(PeopleState());
			state->x_ = 0;		state->y_ = 0;		state->z_ = 0;
			return state;
		}

		// cv::Mat depth_roi_temp(img_depth_, roi);
		cv::Mat tmat(1, roi.width * roi.height, CV_32FC1);
		// = depth_roi_temp.reshape(1, depth_roi_temp.rows * depth_roi_temp.cols);
		
		int cnt = 0;
		for(int i = 0; i < roi.width; i++) {
			for(int j = 0; j < roi.height; j++, cnt++)
				tmat.at<float>(0, cnt) = img_depth_.at<float>(roi.y + j, roi.x + i);
		}

		cv::Mat tmat_sorted;
		cv::sort(tmat, tmat_sorted, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING);
		// in camera coordinate, depth is z!
		pt3[ 2 ] = tmat_sorted.at<float>(floor(cv::countNonZero(tmat_sorted) / 2.0));
		
		btMatrix3x3 cam_rot = cam_state->getRot();
		btVector3 cam_trans = cam_state->getTrans();

		if((pt3[ 2 ] == 0) || (pt3[ 2 ] > 15))
		{
			// need to find alternative way to get the proposals.
			PeopleStatePtr state = boost::make_shared<PeopleState>(PeopleState());
#if 1
			cv::Point im_bottom(rt.x + rt.width / 2, rt.y + rt.height);

			cv::Point3d ray1 = cam_model_.projectPixelTo3dRay(im_top);
			cv::Point3d ray2 = cam_model_.projectPixelTo3dRay(im_bottom);
			assert(ray1.z == ray2.z);

			pt3[ 2 ] = UB_HEIGHT * ray1.z / (ray2.y - ray1.y);

			pt3[ 0 ] = pt3[ 2 ] * ray1.x / ray1.z;
			pt3[ 1 ] = pt3[ 2 ] * ray1.y / ray1.z;

			pt3 = cam_rot * pt3 + cam_trans;
			state->x_ = pt3[ 0 ];		state->y_ = pt3[ 1 ];		state->z_ = pt3[ 2 ];
#else
			state->x_ = 0;		state->y_ = 0;		state->z_ = 0;
#endif
			return state;
		}

		// get inverse-projection
		cv::Point3d ray = cam_model_.projectPixelTo3dRay(im_top);
		pt3[ 0 ] = pt3[ 2 ] * ray.x / ray.z;
		pt3[ 1 ] = pt3[ 2 ] * ray.y / ray.z;
		// transform to reference co-ordinate
		pt3 = cam_rot * pt3 + cam_trans;

		PeopleStatePtr state = boost::make_shared<PeopleState>(PeopleState());
		state->x_ = pt3[ 0 ];		state->y_ = pt3[ 1 ];		state->z_ = pt3[ 2 ];
		return state;
	}

	GFeatStatePtr ObservationManagerGeneralCam::getInitialGFeatState(int idx, CamStatePtr cam_state)
	{
		std::vector<int>::iterator it = std::find(gfeats_idx_.begin(), gfeats_idx_.end(), idx);
		assert(*it == idx);

		int i = (int)(it - gfeats_idx_.begin());
		cv::Point3f ptd = gfeats_[i];

		GFeatStatePtr feat = getGFeatStateFromPoint(ptd, cam_state);
#if 0
		if(isnan(feat->x_) || isnan(feat->y_) || isnan(feat->z_) || 
		  isinf(feat->x_) || isinf(feat->y_) || isinf(feat->z_)) {
			cam_state->print();
			std::cout << "obs point u: " << ptd.x << " v: " << ptd.y << " d: " << ptd.z << std::endl;
			std::cout << "state point x: " << feat->x_ << " y: " << feat->y_ << " z: " << feat->z_ << std::endl;
			my_assert(0);
		}
#endif
		return feat;
	}

	GFeatStatePtr ObservationManagerGeneralCam::getGFeatStateFromPoint(cv::Point3f &ptd, CamStatePtr cam_state)
	{
		// there is no depth.. no information at all..
		assert(ptd.z > 0.0);
		if(ptd.z == 0.0) {
			return GFeatStatePtr();
		}
		
		// there is a depth! let's get a estimated 3d point!
		cv::Point im_pt(ptd.x, ptd.y);
		cv::Point3d ray = cam_model_.projectPixelTo3dRay(im_pt);

		btMatrix3x3 cam_rot = cam_state->getRot();
		btVector3 cam_trans = cam_state->getTrans();
		
		btVector3 pti;
		pti[ 2 ] = ptd.z; // depth
		pti[ 0 ] = pti[ 2 ] * ray.x / ray.z;
		pti[ 1 ] = pti[ 2 ] * ray.y / ray.z;

		btVector3 pt3;
		pt3 = cam_rot * pti + cam_trans;

		GFeatStatePtr state = boost::make_shared<GFeatState>(GFeatState());
		state->x_ = pt3[ 0 ];		state->y_ = pt3[ 1 ];		state->z_ = pt3[ 2 ];

		if(isnan(state->x_) || isnan(state->y_) || isnan(state->z_) || 
		  isinf(state->x_) || isinf(state->y_) || isinf(state->z_)) {
			std::cout << "im_pt " << im_pt.x << " " << im_pt.y << std::endl;
			std::cout << "ray " << ray.x << " " << ray.y  << " " << ray.z << std::endl;

			std::cout << "in Image Coordinate:" << std::endl;
			for(int i = 0; i < 3; i++) {
				std::cout << pti[i] << " ";
			}
			std::cout << std::endl;

			std::cout << "in World Coordinate :" << std::endl;
			for(int i = 0; i < 3; i++) {
				std::cout << pt3[i] << " ";
			}
			std::cout << std::endl;

			cam_state->print();
			std::cout << "rotation:" << std::endl;
			for(int i = 0; i < 3; i++) {
				for(int j = 0; j < 3; j++) {
					std::cout << cam_rot[i][j] << " ";
				}
				std::cout << std::endl;
			}

			std::cout << "trans:" << std::endl;
			for(int i = 0; i < 3; i++) {
				std::cout << cam_trans[i] << " ";
			}
			std::cout << std::endl;
		}

		return state;
	}

	double	ObservationManagerGeneralCam::getPeopleConfidence(PeopleStatePtr ped_state, CamStatePtr cam_state, std::string type)
	{
		double ret = 0;
		cv::Rect rt;
		double depth = ped_state->z_;

		// assuming z is height direction!!!
		if((ped_state->z_ < min_height_) || (ped_state->z_ > max_height_))
			return obs_lkhood_out_of_height_;

		// get image projection.
		rt = getRectFromState(ped_state, cam_state);

		btVector3 pt3;
		pt3[ 0 ] = ped_state->x_; 	pt3[ 1 ] = ped_state->y_;	pt3[ 2 ] = ped_state->z_;

		btMatrix3x3 cam_rot = cam_state->getRot();
		btVector3 cam_trans = cam_state->getTrans();

		// get point in camera co-ordinate
		pt3 = cam_rot.inverse() * (pt3 - cam_trans);
		depth = pt3 [ 2 ];

		if(depth < 0.5) { // not possible to detect anyway..
			return obs_lkhood_out_of_height_;
		}

		if(type == std::string("all")) {
			// iterate over all observation nodes_
			std::vector<ObservationNode*>::iterator it;
			for(it = nodes_.begin(); it < nodes_.end(); it++)
#if 1
				ret += soft_max((*it)->getConfidence(rt, depth), 4.0f);
#else
				ret += (*it)->getConfidence(rt, depth);
#endif
		}
		else {
			std::vector<ObservationNode*>::iterator it;
			for(it = nodes_.begin(); it < nodes_.end(); it++)
			{
				if((*it)->getType() == type)
#if 1
					ret += soft_max((*it)->getConfidence(rt, depth), 4.0f);
#else
					ret += (*it)->getConfidence(rt, depth);
#endif
			}
		}

		ret *= total_weight_;

		return ret;
	}

	double	ObservationManagerGeneralCam::getGFeatConfidence(GFeatStatePtr feat_state, int feat_idx, CamStatePtr cam_state, std::string type)
	{
		cv::Point3f proj = getPointFromState(feat_state, cam_state);
		// assert(proj.z != 0.0);
		cv::Point3f obs = gfeats_[feat_idx];
		// temp parameters
		// double gfeat_sigma_u_ = 2, gfeat_sigma_v_ = 2, gfeat_sigma_d_ = 0.1 * proj.z;
		double ret = log_gaussian_prob(obs.x, proj.x, gfeat_sigma_u_);
		ret += log_gaussian_prob(obs.y, proj.y, gfeat_sigma_v_);
		ret += log_gaussian_prob(obs.z, proj.z, gfeat_sigma_d_ * proj.z);
		// returning log(P(feat, valid | obs) / P(feat, invalid | obx))
		ret -= log_gaussian_prob(2 * gfeat_sigma_u_, 0.0, gfeat_sigma_u_)
				+ log_gaussian_prob(2 * gfeat_sigma_v_, 0.0, gfeat_sigma_v_)
				+ log_gaussian_prob(2 * gfeat_sigma_d_, 0.0, gfeat_sigma_d_ * proj.z);
		
		// remove features with nan 
		if(isnan(ret)) return -100.0f;
		my_assert(!isnan(ret));
		return ret;
	}

	void ObservationManagerGeneralCam::preprocess()
	{
		ObservationManager::preprocess();

		assert(feat_tracker_);
		// feat_tracker_->setVideoFile("/home/wgchoi/feat_tracker_test.avi");
		feat_tracker_->setDetectorType("SURF");
		feat_tracker_->setNewImage(img_mono_, time_sec_);
		feat_tracker_->processTracking();
		// order the features!!!
		// assert(0);
	}

	void ObservationManagerGeneralCam::setData(const void *data, const std::string &type)
	{
		// need the depth image to get 3d point from rect.
		if(type == "image_mono") img_mono_ = *(cv::Mat*)data;
		else if(type == "feat_tracker") feat_tracker_ = (FeatTracker*)data;

		ObservationManager::setData(data, type);
	}
	
	// set ordered feats
	// and return the list of features to be deleted 
	std::vector<int> ObservationManagerGeneralCam::preprocessFeats(const std::vector<int>& prev_feats_idx, const int max_feats, const::std::vector<cv::Rect> &targets)
	{
		std::vector<int> deleted_feats;

		std::vector<int> current_feat_idx;
		std::vector<float> responses;
		std::vector<cv::Point2f> feat_pts;

		feat_tracker_->get_features(time_sec_, feat_pts, responses, current_feat_idx);
#if 0
		std::cout << "responses ";
		for(size_t i = 0; i < current_feat_idx.size(); i++)
		{
			std::cout << current_feat_idx[i] << ":" << responses[i] << " ";
		}
		std::cout << std::endl;
#endif
		gfeats_.clear(); // clear all features;
		gfeats_idx_.clear(); // clear all features;
		// find matches
		int n = 0;
		for(size_t i = 0; i < prev_feats_idx.size(); i++) {
			std::vector<int>::iterator it;
			it = std::find(current_feat_idx.begin(), current_feat_idx.end(), prev_feats_idx[i]);
			// if found.. 
			if(*it == prev_feats_idx[i]) {
				// add the features.. 
				int idx = (int)(it - current_feat_idx.begin());
				float x = feat_pts[idx].x;
				float y = feat_pts[idx].y;
				float depth = img_depth_.at<float>((int)y, (int)x);
				if(depth > 0.0 && !in_any_rect(targets, cv::Point2f(x, y))) {
					cv::Point3f feat(x, y, depth);
					gfeats_.push_back(feat);
					gfeats_idx_.push_back(*it);
					n++;
				}
				else {
					deleted_feats.push_back((int)i);
				}
				// remove already selected features.. 
				current_feat_idx.erase(current_feat_idx.begin() + idx);
				feat_pts.erase(feat_pts.begin() + idx);
				responses.erase(responses.begin() + idx);
			}
			else {
				// std::cout << "no feature found for " << prev_feats_idx[i] << ":" << i << std::endl;
				deleted_feats.push_back((int)i);
			}

			if(n >= max_feats) break;
		}
		
		// add new features!!! 
		while(n < max_feats && current_feat_idx.size() > 0) {
			std::vector<float>::iterator it;
			it = std::max_element(responses.begin(), responses.end());
			int idx = (int)(it - responses.begin());
			// std::cout << "selecting " << idx << ":" << responses[idx] << "=" << *it << std::endl;
			float x = feat_pts[idx].x;
			float y = feat_pts[idx].y;
			float depth = img_depth_.at<float>((int)y, (int)x);
			if(depth > 0.0 && !in_any_rect(targets, cv::Point2f(x, y))) {
				cv::Point3f feat(x, y, depth);
				gfeats_.push_back(feat);
				gfeats_idx_.push_back(current_feat_idx[idx]);
				n++;
			}
			// remove already selected features.. 
			current_feat_idx.erase(current_feat_idx.begin() + idx);
			feat_pts.erase(feat_pts.begin() + idx);
			responses.erase(responses.begin() + idx);
		}
		assert(current_feat_idx.size() == feat_pts.size());
		assert(responses.size() == feat_pts.size());
#if 1
		for(size_t i = 0; i < gfeats_.size(); i++) {
			my_assert(gfeats_[i].z > 0.0f);
		}
#endif
		return deleted_feats;
	}
	
	CamStatePtr ObservationManagerGeneralCam::getInitialCamera(bool init_frame)
	{
		CamStatePtr state = boost::make_shared<CamState>(CamState());
		if(init_frame) {
			state->x_ = cam_to_ref_trans_[0];
			state->y_ = cam_to_ref_trans_[1];
			state->z_ = cam_to_ref_trans_[2];

			double roll, pitch, yaw;

			cam_to_ref_rot_.getRPY(roll, pitch, yaw);
			std::cout << "roll : " << setprecision(3) << roll << " pitch : " << pitch << " yaw : " << yaw << std::endl;

			state->pitch_ = pitch;
			state->yaw_ = yaw;
			state->roll_ = roll;
		}
		else
		{
			// need to implement RANSAC to get initialization of the camera motion
			assert(0);
		}
		return state;
	}

	double ObservationManagerGeneralCam::getDepthFromState(PeopleStatePtr state, CamStatePtr cam_state)
	{
		btVector3 pt3;
		pt3[ 0 ] = state->x_; 	pt3[ 1 ] = state->y_;		pt3[ 2 ] = state->z_;
		btMatrix3x3 cam_rot = cam_state->getRot();
		btVector3 cam_trans = cam_state->getTrans();
		// get point in camera co-ordinate
		pt3 = cam_rot.inverse() * (pt3 - cam_trans);
		// get point in camera co-ordinate
		// pt3 = ref_to_cam_rot_ * pt3 + ref_to_cam_trans_;
		return pt3[ 2 ];
	}

	bool ObservationManagerGeneralCam::debug_check_projections(PeopleStatePtr state)
	{
		CamStatePtr cam = getInitialCamera(true);

		std::cout << "org " << state->x_ << ", " << state->y_ << ", " << state->z_ << std::endl;

		cv::Rect rt1 = getRectFromState(state, cam);
		cv::Rect rt2 = ObservationManager::getRectFromState(state);

		std::cout << "new projection : " << rt1.x << ", " << rt1.y << ", " << rt1.width << ", " << rt1.height << std::endl;
		std::cout << "old projection : " << rt2.x << ", " << rt2.y << ", " << rt2.width << ", " << rt2.height << std::endl;
		
		PeopleStatePtr temp1 = getPeopleStateFromRect(rt1, cam);
		PeopleStatePtr temp2 = ObservationManager::getStateFromRect(rt2);
		
		std::cout << "temp1 " << temp1->x_ << ", " << temp1->y_ << ", " << temp1->z_ << std::endl;
		std::cout << "temp2 " << temp2->x_ << ", " << temp2->y_ << ", " << temp2->z_ << std::endl;

		return (rt1.x == rt2.x)
				& (rt1.y == rt2.y)
				& (rt1.width == rt2.width)
				& (rt1.height == rt2.height);
	}

	std::vector<PeopleStatePtr> ObservationManagerGeneralCam::getDetections(const std::vector<PeopleStatePtr> &proposals, CamStatePtr cam_state)
	{
		std::vector<cv::Rect> rts;
		std::vector<PeopleStatePtr> dets;

		std::cout << "3D proposals : " << rts.size() << std::endl;
		// get detections
		rts = getBoundingBoxes(rts, 0.0f);
		std::cout << " + det proposals : " << rts.size() << std::endl;
		// get 3D location
		for(size_t i = 0; i < rts.size(); i++)
		{
			PeopleStatePtr temp = getPeopleStateFromRect(rts[i], cam_state);
			if((temp->z_ > min_height_) && (temp->z_ < max_height_)) {
				dets.push_back(temp);
			}
		}
		for(size_t i = 0; i < proposals.size(); i++)
			dets.push_back(proposals[i]);

		std::cout << " => 3D det proposals : " << dets.size() << std::endl;

		// group all
		std::vector<int> labels;
		cv::partition(dets, labels, SimilarProposals(0.2));
		
		// get maximum confidence points in each group
		std::vector<PeopleStatePtr> rets;
		for(size_t i = 0; i < labels.size(); i++) {
			int maxidx = -1;
			double maxconf = -40.0f; // threshold for proposals
			for(size_t j = 0; j < labels.size(); j++) {
				if(labels[j] == (int)i)
				{
					double conf = getPeopleConfidence(dets[j], cam_state);
					if(conf > maxconf) {
						maxidx = j;
						maxconf = conf;
					}
				}
			}
			if(maxidx >= 0) {
				rets.push_back(dets[maxidx]);
			}
		}
		std::cout << " => final 3D det proposals : " << rets.size() << std::endl;

		return rets;
	}

	cv::Mat	ObservationManagerGeneralCam::getPeopleConfidenceMap(double z, CamStatePtr cam_state, std::string type)
	{
		cv::Mat ret(200, 200, CV_32FC1);
		btVector3 pt;
		btVector3 pt0 = cam_state->getTrans();

		PeopleStatePtr state = boost::make_shared<PeopleState>(PeopleState());
		
		for(int i = 0; i < 200; i++) {
			for(int j = 0; j < 200; j++) {
				double dx = (i - 100) * 0.05;
				double dz = j * 0.05;
				
				pt[0] = dx;
				pt[1] = 0;
				pt[2] = dz;

				pt = pt0 + cam_state->getRot() * pt;
				pt[2] = z; // fix the height
				
				state->x_ = pt[0];
				state->y_ = pt[1];
				state->z_ = pt[2];
				ret.at<float>(i, j) = (float)getPeopleConfidence(state, cam_state, type);
			}
		}

		return ret;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Observation Manager for Simplified Camera Model
	////////////////////////////////////////////////////////////////////////////////////////////////////////
}; // Namespace
