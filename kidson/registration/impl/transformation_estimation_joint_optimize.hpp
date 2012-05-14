/*
 * TransformationEstimationJointOptimize.cpp
 *
 *  Created on: may 8, 2012
 *      Author: rossk
 */

//#include "transformation_estimation_joint_optimize.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
TransformationEstimationJointOptimize<PointSource, PointTarget>::setWeights(float denseWeight, float visualWeight, float handleWeight) {
	// Limit alpha_ to range [0,1]
	if (denseWeight>1) denseCloudWeight = 1;
	else if (denseWeight<0) denseCloudWeight = 0;
	else denseCloudWeight = denseWeight;

	if (visualWeight>1) visualFeatureWeight = 1;
	else if (visualWeight<0) visualFeatureWeight = 0;
	else visualFeatureWeight = visualWeight;

	if (handleWeight>1) handleFeatureWeight = 1;
	else if (handleWeight<0) handleFeatureWeight = 0;
	else handleFeatureWeight = handleWeight;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
TransformationEstimationJointOptimize<PointSource, PointTarget>::setCorrespondecesDFP ( pcl::Correspondences correspondences_dfp ) {
	const int nr_correspondences_dfp = (int)correspondences_dfp.size();
	indices_src_dfp_.resize(nr_correspondences_dfp);
	indices_tgt_dfp_.resize(nr_correspondences_dfp);
	for (int i = 0; i < nr_correspondences_dfp; ++i)
	{
		indices_src_dfp_[i] = correspondences_dfp[i].index_query;
		indices_tgt_dfp_[i] = correspondences_dfp[i].index_match;
	}
	// Update flags
	indices_src_dfp_set_ = true;
	indices_tgt_dfp_set_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
TransformationEstimationJointOptimize<PointSource, PointTarget>::setCorrespondecesDFP( std::vector<int> &indices_src_dfp_arg,
																				std::vector<int> &indices_tgt_dfp_arg ) {
	// Copy data
	indices_src_dfp_ = indices_src_dfp_arg;
	indices_tgt_dfp_ = indices_tgt_dfp_arg;
	// Update flags
	indices_src_dfp_set_ = true;
	indices_tgt_dfp_set_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
TransformationEstimationJointOptimize<PointSource, PointTarget>::setWeightsDFP (std::vector<float> weights_dfp_arg) {
	weights_dfp_ = weights_dfp_arg;
	weights_dfp_set_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
TransformationEstimationJointOptimize<PointSource, PointTarget>::estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Eigen::Matrix4f &transformation_matrix) {
	// TODO
	;
}

template <typename PointSource, typename PointTarget> inline void
TransformationEstimationJointOptimize<PointSource, PointTarget>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const pcl::Correspondences &correspondences,
    Eigen::Matrix4f &transformation_matrix) {
	// TODO
	;
}

template <typename PointSource, typename PointTarget> inline void
TransformationEstimationJointOptimize<PointSource, PointTarget>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Eigen::Matrix4f &transformation_matrix) {
	// TODO
	;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
TransformationEstimationJointOptimize<PointSource, PointTarget>::estimateRigidTransformation (
		const pcl::PointCloud<PointSource> &cloud_src,
		const std::vector<int> &indices_src,
		const std::vector<int> &indices_src_dfp,
		const pcl::PointCloud<PointTarget> &cloud_tgt,
		const std::vector<int> &indices_tgt,
		const std::vector<int> &indices_tgt_dfp,
		float alpha_arg,
		Eigen::Matrix4f &transformation_matrix)
{
	if (indices_src.size () != indices_tgt.size ())
	{
		PCL_ERROR ("[pcl::registration::TransformationEstimationJointOptimize::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)indices_src.size (), (unsigned long)indices_tgt.size ());
		return;
	}

	if (indices_src_dfp.size () != indices_tgt_dfp.size ())
	{
		PCL_ERROR ("[pcl::registration::TransformationEstimationJointOptimize::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)indices_src_dfp.size (), (unsigned long)indices_tgt_dfp.size ());
		return;
	}

	if ( (indices_src.size () + indices_tgt_dfp.size () )< 4)     // need at least 4 samples
	{
		PCL_ERROR ("[pcl::registration::TransformationEstimationJointOptimize] ");
		PCL_ERROR ("Need at least 4 points to estimate a transform! Source and target have %lu points!",
				(unsigned long)indices_src.size ());
		return;
	}

	// If no warp function has been set, use the default (WarpPointRigid6D)
	if (!warp_point_)
		warp_point_.reset (new pcl::WarpPointRigid6D<PointSource, PointTarget>);

	int n_unknowns = warp_point_->getDimension ();  // get dimension of unknown space
	int num_p = indices_src.size ();
	int num_dfp = indices_src_dfp.size ();
	Eigen::VectorXd x(n_unknowns);
	x.setConstant (n_unknowns, 0);

	// Set temporary pointers
	tmp_src_ = &cloud_src;
	tmp_tgt_ = &cloud_tgt;
	tmp_idx_src_ = &indices_src;
	tmp_idx_tgt_ = &indices_tgt;
	tmp_idx_src_dfp_ = &indices_src_dfp;
	tmp_idx_tgt_dfp_ = &indices_tgt_dfp;


	// TODO: CHANGE NUMBER OF VALUES TO NUM_P + NUM_DFP
	OptimizationFunctor functor (n_unknowns, 1, num_p, num_dfp, this); // Initialize functor
	Eigen::NumericalDiff<OptimizationFunctor> num_diff (functor); // Add derivative functionality
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor> > lm (num_diff);
	int info = lm.minimize (x); // Minimize cost

	// Compute the norm of the residuals
//	PCL_DEBUG ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation]");
//	PCL_DEBUG ("LM solver finished with exit code %i, having a residual norm of %g. \n", info, lm.fvec.norm ());
//	PCL_DEBUG ("Final solution: [%f", x[0]);
	std::cout << "[pcl::registration::TransformationEstimationJointOptimize::estimateRigidTransformation]" << std::endl;
	std::cout << "LM solver finished with exit code " << info <<", having a residual norm of " << lm.fvec.norm () << std::endl;
	std::cout << "Final solution: " << x[0] << std::endl;
	for (int i = 1; i < n_unknowns; ++i)
		PCL_DEBUG (" %f", x[i]);
	PCL_DEBUG ("]\n");

	// Return the correct transformation
	Eigen::VectorXf params = x.cast<float> ();
	warp_point_->setParam (params);
	transformation_matrix = warp_point_->getTransform ();

	tmp_src_ = NULL;
	tmp_tgt_ = NULL;
	tmp_idx_src_ = tmp_idx_tgt_ = NULL;
}

template <typename PointSource, typename PointTarget> inline void
TransformationEstimationJointOptimize<PointSource, PointTarget>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const std::vector<int> &indices_tgt,
    Eigen::Matrix4f &transformation_matrix )
    {
	;
    }

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
TransformationEstimationJointOptimize<PointSource, PointTarget>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const std::vector<int> &handles_indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const std::vector<int> &indices_tgt,
    const std::vector<int> &handles_indices_tgt,
    Eigen::Matrix4f &transformation_matrix )
{
	if (indices_src.size () != indices_tgt.size ())
	{
		PCL_ERROR ("[pcl::registration::TransformationEstimationJointOptimize::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)indices_src.size (), (unsigned long)indices_tgt.size ());
		return;
	}

	if (!((indices_src_dfp_set_)&&(indices_tgt_dfp_set_)))
	{
		PCL_ERROR ("[pcl::registration::TransformationEstimationJointOptimize::estimateRigidTransformation] Correspondences of distinctive feature points in source and target clouds are not set");
		return;
	}

	if (indices_src_dfp_.size () != indices_tgt_dfp_.size ())
	{
		PCL_ERROR ("[pcl::registration::TransformationEstimationJointOptimize::estimateRigidTransformation] Number or distinctive feature points in source (%lu) differs than target (%lu)!\n", (unsigned long)indices_src_dfp_.size (), (unsigned long)indices_tgt_dfp_.size ());
		return;
	}

	if (weights_dfp_set_ && ((weights_dfp_.size () != indices_src_dfp_.size ())))
	{
		PCL_ERROR ("[pcl::registration::TransformationEstimationJointOptimize::estimateRigidTransformation] Number or distinctive feature point pairs weights of  (%lu) differs than number of distinctive feature points (%lu)!\n", (unsigned long)weights_dfp_.size (), (unsigned long)indices_tgt_dfp_.size ());
		return;
	}

	if ( (indices_src.size () + indices_src_dfp_.size()) < 4)     // need at least 4 samples
	{
		PCL_ERROR ("[pcl::registration::TransformationEstimationJointOptimize] ");
		PCL_ERROR ("Need at least 4 points to estimate a transform! Source and target have %lu points!",
				(unsigned long)indices_src.size ());
		return;
	}

	// If no warp function has been set, use the default (WarpPointRigid6D)
	if (!warp_point_)
		warp_point_.reset (new pcl::WarpPointRigid6D<PointSource, PointTarget>);

	int n_unknowns = warp_point_->getDimension ();  // get dimension of unknown space
	int num_p = indices_src.size ();
	int num_dfp = indices_src_dfp_.size ();
	int num_handle_p = handles_indices_src.size ();
	Eigen::VectorXd x(n_unknowns);
	x.setConstant (n_unknowns, 0);

	// Set temporary pointers to clouds
	tmp_src_ = &cloud_src;
	tmp_tgt_ = &cloud_tgt;
	// ... common points
	tmp_idx_src_ = &indices_src;
	tmp_idx_tgt_ = &indices_tgt;
	// ... DF points
	tmp_idx_src_dfp_ = &indices_src_dfp_;
	tmp_idx_tgt_dfp_ = &indices_tgt_dfp_;
	// ... handles
	tmp_idx_src_handles_ = &handles_indices_src;
	tmp_idx_tgt_handles_ = &handles_indices_tgt;

	//std::cerr << "indices_src_dfp_ size " << indices_src_dfp_.size() << " \n ";
	//std::cerr << "indices_tgt_dfp_ size " << indices_tgt_dfp_.size() << " \n ";

	int info;
	double lm_norm;
	int iter;
	if (weights_dfp_set_) {
		// DF Weights are set
		tmp_dfp_weights_ = &weights_dfp_;
		OptimizationFunctorWithWeights functor (n_unknowns, num_handle_p+num_p+num_dfp, num_p, num_dfp, num_handle_p, this); // Initialize functor
		Eigen::NumericalDiff<OptimizationFunctorWithWeights> num_diff (functor); // Add derivative functionality
		Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctorWithWeights> > lm (num_diff);
		/* From Eigen::  enum  	Status {
		  NotStarted = -2, Running = -1, ImproperInputParameters = 0, RelativeReductionTooSmall = 1,
		  RelativeErrorTooSmall = 2, RelativeErrorAndReductionTooSmall = 3, CosinusTooSmall = 4, TooManyFunctionEvaluation = 5,
		  FtolTooSmall = 6, XtolTooSmall = 7, GtolTooSmall = 8, UserAsked = 9
		} */
		info = lm.minimize (x); // Minimize cost
		lm_norm = lm.fvec.norm ();
		iter = lm.iter;
	} else {
		// DF Weights are not set
		OptimizationFunctor functor (n_unknowns, num_p+num_dfp+num_handle_p, num_p, num_dfp, num_handle_p, this); // Initialize functor
		Eigen::NumericalDiff<OptimizationFunctor> num_diff (functor); // Add derivative functionality

		Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor> > lm (num_diff);
		info = lm.minimize (x); // Minimize cost
		lm_norm = lm.fvec.norm ();
		iter = lm.iter;
	}

	// Compute the norm of the residuals
//	PCL_DEBUG ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation]");
//	PCL_DEBUG ("LM solver finished with exit code %i, having a residual norm of %g. \n", info, lm_norm ());
//	PCL_DEBUG ("Final solution: [%f", x[0]);
//	for (int i = 1; i < n_unknowns; ++i)
//		PCL_DEBUG (" %f", x[i]);
//	PCL_DEBUG ("]\n");
/*
	std::cout << "[pcl::registration::TransformationEstimationJointOptimize::estimateRigidTransformation]" << std::endl;
	std::cout << "LM solver finished with exit code " << info <<", having a residual norm of " << lm_norm
			<< ", in iteration "<< iter << std::endl;
*/
	// Return the correct transformation
	Eigen::VectorXf params = x.cast<float> ();
	warp_point_->setParam (params);
	transformation_matrix = warp_point_->getTransform ();
	//std::cout << "	Obtained transform = " << std::endl << transformation_matrix << std::endl;

	tmp_src_ = NULL;
	tmp_tgt_ = NULL;
	tmp_idx_src_ = tmp_idx_tgt_ = NULL;
	tmp_idx_src_dfp_ = tmp_idx_tgt_dfp_ = NULL;
	tmp_idx_src_handles_ = tmp_idx_tgt_handles_ = NULL;
	tmp_weights_ = NULL;
};

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
TransformationEstimationJointOptimize<PointSource, PointTarget>::estimateRigidTransformation (
		const pcl::PointCloud<PointSource> &cloud_src,
		const pcl::PointCloud<PointTarget> &cloud_tgt,
		const pcl::Correspondences &correspondences,
		const pcl::Correspondences &correspondences_dfp,
		float alpha_arg,
		Eigen::Matrix4f &transformation_matrix)
{
	const int nr_correspondences = (int)correspondences.size();
	std::vector<int> indices_src(nr_correspondences);
	std::vector<int> indices_tgt(nr_correspondences);
	for (int i = 0; i < nr_correspondences; ++i)
	{
		indices_src[i] = correspondences[i].index_query;
		indices_tgt[i] = correspondences[i].index_match;
	}

	const int nr_correspondences_dfp = (int)correspondences_dfp.size();
	std::vector<int> indices_src_dfp(nr_correspondences_dfp);
	std::vector<int> indices_tgt_dfp(nr_correspondences_dfp);
	for (int i = 0; i < nr_correspondences_dfp; ++i)
	{
		indices_src_dfp[i] = correspondences_dfp[i].index_query;
		indices_tgt_dfp[i] = correspondences_dfp[i].index_match;
	}
	estimateRigidTransformation(cloud_src, indices_src, indices_src_dfp, cloud_tgt, indices_tgt, indices_tgt_dfp,
			alpha_arg, transformation_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> int
TransformationEstimationJointOptimize<PointSource, PointTarget>::OptimizationFunctor::operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
	const pcl::PointCloud<PointSource> & src_points = *estimator_->tmp_src_;
	const pcl::PointCloud<PointTarget> & tgt_points = *estimator_->tmp_tgt_;
	const std::vector<int> & src_indices = *estimator_->tmp_idx_src_;
	const std::vector<int> & tgt_indices = *estimator_->tmp_idx_tgt_;
	const std::vector<int> & src_indices_dfp = *estimator_->tmp_idx_src_dfp_;
	const std::vector<int> & tgt_indices_dfp = *estimator_->tmp_idx_tgt_dfp_;
	const std::vector<int> & src_indices_handles = *estimator_->tmp_idx_src_handles_;
	const std::vector<int> & tgt_indices_handles = *estimator_->tmp_idx_tgt_handles_;
	const float visualWeight = estimator_->visualFeatureWeight;
	const float denseWeight = estimator_->denseCloudWeight;
	const float handleWeight = estimator_->handleFeatureWeight;

	// Initialize the warp function with the given parameters
	Eigen::VectorXf params = x.cast<float> ();
	estimator_->warp_point_->setParam (params);

	Eigen::Matrix4f curr_transformation_matrix = estimator_->warp_point_->getTransform ();
	//std::cout << "[OptimizationFunctor::operator()] current transform = " << std::endl << curr_transformation_matrix << std::endl;

	//std::cerr << "[error function] before visual points: " << fvec.sum() << "\n";
	// Sum of squared distances of distinctive feature points
//	double diff_value_dfp = 0;
	const double dfp_factor = visualWeight/number_dfp;
	for (int i = 0; i < number_dfp; ++i)
	{
		std::cerr << "[error function] entered visual points loop: " << fvec.sum() << "\n";
		const PointSource & p_src = src_points.points[src_indices_dfp[i]];
		const PointTarget & p_tgt = tgt_points.points[tgt_indices_dfp[i]];

		// Transform the source point based on the current warp parameters
		PointSource p_src_warped;
		estimator_->warp_point_->warpPoint (p_src, p_src_warped);

		// Estimate the distance (cost function)
//		diff_value_dfp += estimator_->computeDistance (p_src_warped, p_tgt);
		fvec[i] = dfp_factor * estimator_->computeDistance (p_src_warped, p_tgt);
	}
	//std::cerr << "[error function] after visual points: " << fvec.sum() << "\n";

	const double p_factor = (denseWeight)/number_p;
	for (int i = 0; i < number_p; ++i)
	{
		const PointSource & p_src = src_points.points[src_indices[i]];
		const PointTarget & p_tgt = tgt_points.points[tgt_indices[i]];

		// Transform the source point based on the current warp parameters
		PointSource p_src_warped;
		estimator_->warp_point_->warpPoint (p_src, p_src_warped);

		// Estimate the distance (cost function)
//		diff_value_p += estimator_->computeDistance (p_src_warped, p_tgt);
		fvec[i+number_dfp] = p_factor * estimator_->computeDistancePointToPlane (p_src_warped, p_tgt);

		if(fvec[i+number_dfp] != fvec[i+number_dfp])
		{
			std::cerr << "i: " << i << " is nan! fvec[i]= " << fvec[i+number_dfp] << "\n";
			std::cerr << "source point       : " << p_src << "\n";
			std::cerr << "source point warped: " << p_src_warped << "\n";
			std::cerr << "target point       : " << p_tgt << "\n";
		}

		//	std::cerr << "fvec point " << (int)i << ":" << fvec[i+number_dfp] << "\n";
	}
	//std::cerr << "[error function] after points and features: " << fvec.sum() << "\n";

	const double handle_factor = (handleWeight)/number_handle_p;
	double totalError = 0.0;
	for (int i = 0; i < number_handle_p; ++i)
	{
		const PointSource & p_src = src_points.points[src_indices_handles[i]];
		const PointTarget & p_tgt = tgt_points.points[tgt_indices_handles[i]];

		// Transform the source point based on the current warp parameters
		PointSource p_src_warped;
		estimator_->warp_point_->warpPoint (p_src, p_src_warped);

		// Estimate the distance (cost function)
//		diff_value_p += estimator_->computeDistance (p_src_warped, p_tgt);
		fvec[i+number_dfp+number_p] = handle_factor * estimator_->computeDistance (p_src_warped, p_tgt);
		totalError += fvec[i+number_dfp+number_p];
		//std::cerr << "i: " << i << " src indice: " << src_indices_handles[i] << " src point: " << p_src_warped
		//                        << " tgt indice: " << tgt_indices_handles[i] << " tgt point: " << p_tgt
		//std::cerr << "i: " << i << " Distance error: " << fvec[i+number_dfp+number_p] << "\n";

		//std::cerr << "fvec handle point " << (int)i << ":" << fvec[i+number_dfp+number_p] << "\n";
	}
	//std::cerr << "[error function] after points and features and handles: " << fvec.sum() << "\n";
	//fvec[0] = totalError;
	//for (int i = 1; i < (number_handle_p + number_p + number_dfp); ++i)
	//{

	//	std::cerr << "i: " << i << "error: " << fvec[i] << "\n";
	//	fvec[i] = 0.0;
	//}
	//std::cerr << "[error functino] fvec total error : " << fvec.sum() << "\n";
	//std::cerr << "[error functino] my   total error : " << totalError << "\n";
	//std::cerr << "##############Other debug info: ##########\n"
	//		<< "Visual weight: " <<  visualWeight << " visual points: " << number_dfp      << " visual factor: " << dfp_factor << "\n"
	//		<< "dense weight:  " <<  denseWeight  << " dense  points: " << number_p        << " dense  factor: " << p_factor << "\n"
	//		<< "handle weight: " <<  handleWeight << " handle points: " << number_handle_p << " handle factor: " << handle_factor << "\n";
	// Divide by number of points
//	diff_value_p = diff_value_p/number_p;

	// Update function value
//	fvec[0] = ((alpha * diff_value_dfp) + ((1-alpha) * diff_value_p))*1000;

	//std::cout << "[OptimizationFunctor::operator()] fvec.blueNorm = " << fvec.blueNorm() << std::endl;

	return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> int
TransformationEstimationJointOptimize<PointSource, PointTarget>::OptimizationFunctorWithWeights::operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
	const pcl::PointCloud<PointSource> & src_points = *estimator_->tmp_src_;
	const pcl::PointCloud<PointTarget> & tgt_points = *estimator_->tmp_tgt_;
	const std::vector<int> & src_indices = *estimator_->tmp_idx_src_;
	const std::vector<int> & tgt_indices = *estimator_->tmp_idx_tgt_;
	const std::vector<int> & src_indices_dfp = *estimator_->tmp_idx_src_dfp_;
	const std::vector<int> & tgt_indices_dfp = *estimator_->tmp_idx_tgt_dfp_;
	const std::vector<float> & weights_dfp = *estimator_->tmp_dfp_weights_;
	const float visualWeight = estimator_->visualFeatureWeight;
	const float denseWeight = estimator_->denseCloudWeight;
	const float handleWeight = estimator_->handleFeatureWeight;

//	  const PointCloud<PointSource> & src_points = *estimator_->tmp_src_;
//	  const PointCloud<PointTarget> & tgt_points = *estimator_->tmp_tgt_;
//	  const std::vector<int> & src_indices = *estimator_->tmp_idx_src_;
//	  const std::vector<int> & tgt_indices = *estimator_->tmp_idx_tgt_;
//
//	  // Initialize the warp function with the given parameters
//	  Eigen::VectorXf params = x.cast<float> ();
//	  estimator_->warp_point_->setParam (params);
//
//	  // Transform each source point and compute its distance to the corresponding target point
//	  for (int i = 0; i < m_values; ++i)
//	  {
//	    const PointSource & p_src = src_points.points[src_indices[i]];
//	    const PointTarget & p_tgt = tgt_points.points[tgt_indices[i]];
//
//	    // Transform the source point based on the current warp parameters
//	    PointSource p_src_warped;
//	    estimator_->warp_point_->warpPoint (p_src, p_src_warped);
//
//	    // Estimate the distance (cost function)
//	    fvec[i] = estimator_->computeDistance (p_src_warped, p_tgt);
//	  }
//	  return (0);

	// Initialize the warp function with the given parameters
	Eigen::VectorXf params = x.cast<float> ();
	estimator_->warp_point_->setParam (params);

	// Sum of squared distances of distinctive feature points
//	double diff_value_dfp = 0;
	const double dfp_factor = visualWeight/number_dfp;
	for (int i = 0; i < number_dfp; ++i)
	{
		const PointSource & p_src = src_points.points[src_indices_dfp[i]];
		const PointTarget & p_tgt = tgt_points.points[tgt_indices_dfp[i]];

		// Transform the source point based on the current warp parameters
		PointSource p_src_warped;
		estimator_->warp_point_->warpPoint (p_src, p_src_warped);

		// Estimate the distance (cost function)
//		diff_value_dfp += weights_dfp[i] * estimator_->computeDistance (p_src_warped, p_tgt);
		fvec[i] = dfp_factor * weights_dfp[i] * estimator_->computeDistance (p_src_warped, p_tgt);
	}
	// Divide by number of points
//	diff_value_dfp = diff_value_dfp/number_dfp;

	// Sum of squared distances of common points
//	double diff_value_p = 0;
	const double p_factor = denseWeight/number_p;
	for (int i = 0; i < number_p; ++i)
	{
		const PointSource & p_src = src_points.points[src_indices[i]];
		const PointTarget & p_tgt = tgt_points.points[tgt_indices[i]];

		// Transform the source point based on the current warp parameters
		PointSource p_src_warped;
		estimator_->warp_point_->warpPoint (p_src, p_src_warped);

		// Estimate the distance (cost function)
//		diff_value_p += estimator_->computeDistance (p_src_warped, p_tgt);

		// TODO: ADD WEIGHTS FOR COMMON POINTS, TOO
		fvec[i+number_dfp] = p_factor * estimator_->computeDistance (p_src_warped, p_tgt);
	}
	// Divide by number of points
//	diff_value_p = diff_value_p/number_p;

	// Update function value
//	fvec[0] = ((alpha * diff_value_dfp) + ((1-alpha) * diff_value_p))*1000;
	return (0);
}
