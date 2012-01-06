#ifndef _OBSERVATION_MANAGER_H_
#define _OBSERVATION_MANAGER_H_

#include <common/ped_state.h>
#include <common/cam_state.h>
#include <common/gfeat_state.h>
#include <observation/ObservationNode.h>

#include <image_geometry/pinhole_camera_model.h>
#include <LinearMath/btTransform.h>

#include <common/FeatTracker.h>

namespace people {
	class ObservationManager
	{
	public:
		ObservationManager();
		virtual ~ObservationManager();

		void 		releaseNodes();
		
		void 		insertObservationNode(ObservationNode* node);
		ObservationNode *getObservationNode(std::string &type);

		virtual void	setCameraModel(image_geometry::PinholeCameraModel &cam_model, btMatrix3x3 ref_to_cam_rot, btVector3 ref_to_cam_trans
																				, btMatrix3x3 cam_to_ref_rot, btVector3 cam_to_ref_trans);
		// these two are totally unsafe!!! be cautious!!
		virtual void	setData(const void *data, const std::string &type);
		virtual void	quaryData(const std::string &name, void *data);

		virtual void		setParameters(const std::string &name, const std::string &value);

		virtual void 		preprocess();

		virtual std::vector<PeopleStatePtr> 	getDetections(const std::vector<PeopleStatePtr> &proposals = std::vector<PeopleStatePtr>());
		virtual std::vector<PeopleStatePtr> 	getDetections2(const std::vector<PeopleStatePtr> &proposals = std::vector<PeopleStatePtr>());
		virtual std::vector<cv::Rect> 		getBoundingBoxes(const std::vector<cv::Rect> &proposals = std::vector<cv::Rect>(), double eps = 0.2);

		virtual double	getConfidence(PeopleStatePtr state, std::string type = std::string("all")); 
#if 0
		virtual void computeNewSampleObservationLkhood(const SampleInfo &info);
		virtual void initCache(MCMCSamplePtr sample);
		virtual double computeLogObservationLkhood(const SampleInfo &info);
		void updateCache(const SampleInfo &info, bool accepted);
#endif
		virtual PeopleStatePtr		getStateFromRect(cv::Rect &rt);
		virtual cv::Rect		getRectFromState(PeopleStatePtr state);
		virtual double		getDepthFromState(PeopleStatePtr state);

		virtual btVector3 			getCameraOrigin(){ return cam_to_ref_trans_; }
		virtual btMatrix3x3			getCameraBasis(){ return cam_to_ref_rot_; }

		inline double 	getTimeSec() {return time_sec_;};
		inline cv::Mat	getImage() {return img_color_;}
	protected:
		cv::Mat															img_color_;
		cv::Mat															img_depth_;

		double															time_sec_;
		double															total_weight_;
		double															min_height_;
		double															max_height_;

		double															obs_lkhood_out_of_height_;

		std::vector<ObservationNode*> 			nodes_;
		image_geometry::PinholeCameraModel 	cam_model_;

		btVector3														ref_to_cam_trans_;
		btMatrix3x3													ref_to_cam_rot_;
		btVector3														cam_to_ref_trans_;
		btMatrix3x3													cam_to_ref_rot_;
#if 0
		std::vector<double> 								obs_cache_;
		double															one_obs_cache_;
#endif
	};
	
	/* observation manager class that is compatible to camera estimation version */
	class ObservationManagerCamEst : public ObservationManager 
	{
	public:
		ObservationManagerCamEst() { ObservationManager::ObservationManager(); };
		virtual ~ObservationManagerCamEst() { releaseNodes(); };

		// not supported in this class!!! Camera parameters must be provided
		virtual void setCameraModel(image_geometry::PinholeCameraModel &cam_model, btMatrix3x3 ref_to_cam_rot, btVector3 ref_to_cam_trans, btMatrix3x3 cam_to_ref_rot, btVector3 cam_to_ref_trans) {
			ObservationManager::setCameraModel(cam_model, ref_to_cam_rot, ref_to_cam_trans, cam_to_ref_rot, cam_to_ref_trans); 
			// it will be used only as a initialization in testing...
		}
		virtual PeopleStatePtr getStateFromRect(cv::Rect &rt) {	assert(0); return ObservationManager::getStateFromRect(rt); }; 
		virtual cv::Rect getRectFromState(PeopleStatePtr state) { assert(0); return ObservationManager::getRectFromState(state); };
		virtual double getDepthFromState(PeopleStatePtr state) { assert(0); return ObservationManager::getDepthFromState(state); };
		virtual btVector3 			getCameraOrigin(){ assert(0); return cam_to_ref_trans_; }
		virtual btMatrix3x3			getCameraBasis(){ assert(0); return cam_to_ref_rot_; }
		virtual double	getConfidence(PeopleStatePtr state, std::string type = std::string("all")) {assert(0); return ObservationManager::getConfidence(state, type); }; 
	protected:
	};

	class ObservationManagerGeneralCam : public ObservationManagerCamEst
	{
	public:
		ObservationManagerGeneralCam();
		virtual ~ObservationManagerGeneralCam();

		virtual void setCameraModel(image_geometry::PinholeCameraModel &cam_model); // only intrinsic camera model is given!
		// not supported in this class!!! Camera parameters must be provided
		virtual void setCameraModel(image_geometry::PinholeCameraModel &cam_model, btMatrix3x3 ref_to_cam_rot, btVector3 ref_to_cam_trans, btMatrix3x3 cam_to_ref_rot, btVector3 cam_to_ref_trans) {
			ObservationManager::setCameraModel(cam_model, ref_to_cam_rot, ref_to_cam_trans, cam_to_ref_rot, cam_to_ref_trans); 
			// it will be used only as a initialization in testing...
		}

		// camera projection function
		virtual cv::Rect getRectFromState(PeopleStatePtr ped_state, CamStatePtr cam_state);
		virtual cv::Point3f getPointFromState(GFeatStatePtr feat_state, CamStatePtr cam_state);

		// inverse projections related.. used for initialization 
		virtual PeopleStatePtr getPeopleStateFromRect(cv::Rect &rt, CamStatePtr cam_state);
		virtual GFeatStatePtr getGFeatStateFromPoint(cv::Point3f &ptd, CamStatePtr cam_state);
		virtual GFeatStatePtr getInitialGFeatState(int idx, CamStatePtr cam_state);

		virtual double		getDepthFromState(PeopleStatePtr state, CamStatePtr cam_state);

		// getConfidence
		virtual double	getPeopleConfidence(PeopleStatePtr ped_state, CamStatePtr cam_state, std::string type = std::string("all"));
		virtual double	getGFeatConfidence(GFeatStatePtr feat_state, int feat_idx, CamStatePtr cam_state, std::string type = std::string("all"));

		virtual cv::Mat	getPeopleConfidenceMap(double z, CamStatePtr cam_state, std::string type = std::string("all")); 

		virtual void 		preprocess();

		virtual std::vector<int> preprocessFeats(const std::vector<int>& prev_feats_idx, const int max_feats, const std::vector<cv::Rect> &targets = std::vector<cv::Rect>());
		virtual std::vector<int> getFeatsIndex() {return gfeats_idx_;};
		virtual std::vector<cv::Point3f> getAllFeats() {return gfeats_;};

		virtual void	setParameters(const std::string &name, const std::string &value);
		virtual void	setData(const void *data, const std::string &type);

		virtual CamStatePtr getInitialCamera(bool init_frame);

		virtual std::vector<PeopleStatePtr> getDetections(const std::vector<PeopleStatePtr> &proposals, CamStatePtr cam_state);

		bool debug_check_projections(PeopleStatePtr state);
	protected:
		// ordered set of geometric features
		std::vector<cv::Point3f> 	gfeats_;
		std::vector<int>		 	gfeats_idx_;

		cv::Mat						img_mono_;
		FeatTracker*				feat_tracker_;

		double gfeat_sigma_u_; 
		double gfeat_sigma_v_; 
		double gfeat_sigma_d_;
	};
}; // Namespace
#endif // _OBSERVATION_MANAGER_H_
