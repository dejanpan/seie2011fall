#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/boundary.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/keypoints/harris_keypoint3D.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <fstream>
#include <iostream>

#include <math.h>

#define FPS_CALC_BEGIN                          \
    static double duration = 0;                 \
    double start_time = pcl::getTime ();        \

#define FPS_CALC_END(_WHAT_)                    \
  {                                             \
    double end_time = pcl::getTime ();          \
    static unsigned count = 0;                  \
    if (++count == 10)                          \
    {                                           \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(duration) << " Hz" <<  std::endl; \
      count = 0;                                                        \
      duration = 0.0;                                                   \
    }                                           \
    else                                        \
    {                                           \
      duration += end_time - start_time;        \
    }                                           \
  }

unsigned char colormap_ [36] = { 255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0, 255, 0, 255, 0, 255, 255, 127, 0, 0, 0, 127, 0, 0, 0, 127,127, 127, 0, 127, 0, 127, 0, 127, 127 };


bool comparison (pcl::PointXYZI i,pcl::PointXYZI j) {
	 return (i.intensity > j.intensity); }

bool comparison_curv (pcl::Normal i,pcl::Normal j) {
	 return (i.curvature > j.curvature); }

using namespace pcl::tracking;

template<typename PointType>
class OpenNISegmentTracking {
public:
	//typedef pcl::PointXYZRGBANormal RefPointType;
	typedef pcl::PointXYZRGBA RefPointType;
	//typedef pcl::PointXYZ RefPointType;
	typedef ParticleXYZRPY ParticleT;

	typedef pcl::PointCloud<PointType> Cloud;
	typedef pcl::PointCloud<RefPointType> RefCloud;
	typedef typename RefCloud::Ptr RefCloudPtr;
	typedef typename RefCloud::ConstPtr RefCloudConstPtr;
	typedef typename Cloud::Ptr CloudPtr;
	typedef typename Cloud::ConstPtr CloudConstPtr;
	//typedef KLDAdaptiveParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
	//typedef KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> ParticleFilter;
	//typedef ParticleFilterOMPTracker<RefPointType, ParticleT> ParticleFilter;
	typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
	typedef typename ParticleFilter::CoherencePtr CoherencePtr;
	typedef typename pcl::search::KdTree<PointType> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;
	OpenNISegmentTracking(const std::string& device_id, int thread_nr,
			double downsampling_grid_size, bool use_convex_hull,
			bool visualize_non_downsample, bool visualize_particles,
			bool use_fixed, int trackers_number) :
			viewer_("PCL OpenNI Tracking Viewer"), device_id_(device_id), new_cloud_(
					false), ne_(thread_nr), counter_(0), use_convex_hull_(
					use_convex_hull), visualize_non_downsample_(
					visualize_non_downsample), visualize_particles_(
					visualize_particles), downsampling_grid_size_(
					downsampling_grid_size) {
		KdTreePtr tree(new KdTree(false));
		ne_.setSearchMethod(tree);
		ne_.setRadiusSearch(0.03);






		a_file_.open("poses_final.txt");
		//here
		//std::vector<double> default_step_covariance = std::vector<double>(6,
		//		0.015 * 0.015);
		std::vector<double> default_step_covariance = std::vector<double>(6,
				0.005 * 0.015);
		default_step_covariance[3] *= 40.0;
		default_step_covariance[4] *= 40.0;
		default_step_covariance[5] *= 40.0;
//		default_step_covariance[0] *=50.0;

		std::cerr << "step covariance: " << default_step_covariance.size()
				<< std::endl;

		std::vector<double> initial_noise_covariance = std::vector<double>(6,
				0.00001);
		std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

		for (int i=0; i<trackers_number; i++){

		if (use_fixed) {
			boost::shared_ptr<ParticleFilterOMPTracker<RefPointType, ParticleT> > tracker(
					new ParticleFilterOMPTracker<RefPointType, ParticleT>(
							thread_nr));
			tracker_ = tracker;

		} else {
			boost::shared_ptr<
					KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker(
					new KLDAdaptiveParticleFilterOMPTracker<RefPointType,
							ParticleT>(thread_nr));
			//here
			//tracker->setMaximumParticleNum(500);
			tracker->setMaximumParticleNum(1000);

			tracker->setDelta(0.99);
			tracker->setEpsilon(0.2);
			ParticleT bin_size;
			bin_size.x = 0.1;
			bin_size.y = 0.1;
			bin_size.z = 0.1;
			bin_size.roll = 0.1;
			bin_size.pitch = 0.1;
			bin_size.yaw = 0.1;
			tracker->setBinSize(bin_size);
			tracker_ = tracker;
		}

		tracker_->setTrans(Eigen::Affine3f::Identity());
		tracker_->setStepNoiseCovariance(default_step_covariance);
		tracker_->setInitialNoiseCovariance(initial_noise_covariance);
		tracker_->setInitialNoiseMean(default_initial_mean);
		tracker_->setIterationNum(1);

		//here

//		tracker_->setParticleNum(400);
		tracker_->setParticleNum(800);

		tracker_->setResampleLikelihoodThr(0.00);
		tracker_->setUseNormal(false);

		//here
		tracker_->setMotionRatio(0.2);

		// setup coherences
		ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence =
				ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr(
						new ApproxNearestPairPointCloudCoherence<RefPointType>());
		// NearestPairPointCloudCoherence<RefPointType>::Ptr coherence = NearestPairPointCloudCoherence<RefPointType>::Ptr
		//   (new NearestPairPointCloudCoherence<RefPointType> ());

		boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence =
				boost::shared_ptr<DistanceCoherence<RefPointType> >(
						new DistanceCoherence<RefPointType>());
		coherence->addPointCoherence(distance_coherence);

		boost::shared_ptr<HSVColorCoherence<RefPointType> > color_coherence =
				boost::shared_ptr<HSVColorCoherence<RefPointType> >(
						new HSVColorCoherence<RefPointType>());
		color_coherence->setWeight(0.1);
		coherence->addPointCoherence(color_coherence);

		//boost::shared_ptr<pcl::search::KdTree<RefPointType> > search (new pcl::search::KdTree<RefPointType> (false));
		boost::shared_ptr<pcl::search::Octree<RefPointType> > search(
				new pcl::search::Octree<RefPointType>(0.01));
		//boost::shared_ptr<pcl::search::OrganizedNeighbor<RefPointType> > search (new pcl::search::OrganizedNeighbor<RefPointType>);
		coherence->setSearchMethod(search);
		coherence->setMaximumDistance(0.01);
		tracker_->setCloudCoherence(coherence);


		tracker_vector_.push_back(tracker_);
		}

	}

	bool drawParticles(pcl::visualization::PCLVisualizer& viz) {


		for (uint track=0; track < tracker_vector_.size(); track++) {
		bool drawParticles= true;
		ParticleFilter::PointCloudStatePtr particles = tracker_vector_[track]->getParticles();
		//ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
		if (particles) {
			if (visualize_particles_) {
				pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(
						new pcl::PointCloud<pcl::PointXYZ>());
				for (size_t i = 0; i < particles->points.size(); i++) {
					pcl::PointXYZ point;

					point.x = particles->points[i].x;
					point.y = particles->points[i].y;
					point.z = particles->points[i].z;
					particle_cloud->points.push_back(point);
				}

				{
					if (drawParticles){

						pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color(
														particle_cloud, colormap_[3*track], colormap_[3*track+1], colormap_[3*track+2]);
//					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color(particle_cloud, 250, 99,71);
					 if (!viz.updatePointCloud(particle_cloud, blue_color,
					 "particle cloud" +track))
					 viz.addPointCloud(particle_cloud, blue_color,
					 "particle cloud" +track);
					}
				}
			}

//			return true;
		} else {
			PCL_WARN("no particles\n");
			return false;
		}} return true;
	}

	void drawResult(pcl::visualization::PCLVisualizer& viz) {

		for (uint track=0; track < tracker_vector_.size(); track++) {


		ParticleXYZRPY result = tracker_vector_[track]->getResult();
		bool non_tracking=false;
		float row,pitch,yaw,x,y,z;


		if (non_tracking)
		{
			if (counter_<13){

				Eigen::Affine3f transformation = tracker_vector_[track]->toEigenMatrix(result);
				// move a little bit for better visualization
				transformation.translation() += Eigen::Vector3f(0.0, 0.0, -0.005);
				transformation_=transformation;
				}
		}
		else
		{
			Eigen::Affine3f transformation = tracker_vector_[track]->toEigenMatrix(result);
							// move a little bit for better visualization
							transformation.translation() += Eigen::Vector3f(0.0, 0.0, -0.005);
							transformation_=transformation;

//			pcl::getTransformation(x,y,z,row,pitch,yaw,transformation);

		}


		RefCloudPtr result_cloud(new RefCloud());

		if (!visualize_non_downsample_)
			pcl::transformPointCloud<RefPointType>(
					*(tracker_vector_[track]->getReferenceCloud()), *result_cloud,
					transformation_);
		else
//			pcl::transformPointCloud<RefPointType>(*reference_, *result_cloud,
//					transformation_);
		pcl::transformPointCloud<RefPointType>(*(tracker_vector_[track]->getReferenceCloud()), *result_cloud,
							transformation_);


		Eigen::Vector4f center;
		pcl::compute3DCentroid<RefPointType>(*result_cloud, center);

		std::stringstream ss;
					ss <<track;

		//a_file_<<ss.str();
		a_file_<< center[0] << " " << center[1] << " " << center[2] << std::endl;
//		a_file_<< x << " " << y << " " << z <<" " << row << " " << pitch <<  " " << yaw <<std::endl;
		std::cerr<<"center: "<<center<<std::endl;




		{
//			pcl::visualization::PointCloudColorHandlerCustom<RefPointType> red_color(
//					result_cloud, 0, 0, 255);

			//if (track<12)
			pcl::visualization::PointCloudColorHandlerCustom<RefPointType> red_color(
								result_cloud, colormap_[3*track], colormap_[3*track+1], colormap_[3*track+2]);

			std::cerr<<"track: "<<track<<std::endl;




			if (!viz.updatePointCloud(result_cloud, red_color, ss.str()))
				viz.addPointCloud(result_cloud, red_color, ss.str());

			//points' size
			 viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, ss.str());

		}
}
	}

	void viz_cb(pcl::visualization::PCLVisualizer& viz) {
		boost::mutex::scoped_lock lock(mtx_);

		if (!cloud_pass_) {
			boost::this_thread::sleep(boost::posix_time::seconds(1));
			return;
		}

		if (new_cloud_ && cloud_pass_downsampled_) {
			CloudPtr cloud_pass;
			if (!visualize_non_downsample_)
				cloud_pass = cloud_pass_downsampled_;
			else
				cloud_pass = cloud_pass_;

			if (!viz.updatePointCloud(cloud_pass, "cloudpass")) {
				viz.addPointCloud(cloud_pass, "cloudpass");
				viz.resetCameraViewpoint("cloudpass");
			}
		}

		if (new_cloud_ && reference_) {
			bool ret = drawParticles(viz);
			if (ret) {
				drawResult(viz);

				// draw some texts
//
//				viz.removeShape("N");
//				viz.addText(
//						(boost::format("number of Reference PointClouds: %d")
//								% tracker_->getReferenceCloud()->points.size()).str(),
//						10, 20, 20, 1.0, 1.0, 1.0, "N");
//
//				viz.removeShape("M");
//				viz.addText(
//						(boost::format("number of Measured PointClouds:  %d")
//								% cloud_pass_downsampled_->points.size()).str(),
//						10, 40, 20, 1.0, 1.0, 1.0, "M");
//
//				viz.removeShape("tracking");
//				viz.addText(
//						(boost::format("tracking:        %f fps")
//								% (1.0 / tracking_time_)).str(), 10, 60, 20,
//						1.0, 1.0, 1.0, "tracking");
//
//				viz.removeShape("downsampling");
//				viz.addText(
//						(boost::format("downsampling:    %f fps")
//								% (1.0 / downsampling_time_)).str(), 10, 80, 20,
//						1.0, 1.0, 1.0, "downsampling");
//
//				viz.removeShape("computation");
//				viz.addText(
//						(boost::format("computation:     %f fps")
//								% (1.0 / computation_time_)).str(), 10, 100, 20,
//						1.0, 1.0, 1.0, "computation");

				for (uint track=0; track < tracker_vector_.size(); track++) {
//						tracker_vector_[track]->getParticles();
					std::stringstream ss;
					ss <<track;

				viz.removeShape("particles"+ss.str());
				viz.addText(
						(boost::format("particles:     %d")
								% (tracker_vector_[track]->getParticles()->points.size())).str(),
						10, 120*track, 20, 1.0, 1.0, 1.0, "particles"+ss.str());
				}


//				viz.addText("Particle filtering-based tracking of 3D lines and corners. ",20,60,23,1.0,1.0,1.0,"title"+counter_);
			}
		}
		new_cloud_ = false;
	}

	void filterPassThrough(const CloudConstPtr &cloud, Cloud &result) {
		FPS_CALC_BEGIN;
		pcl::PassThrough<PointType> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 10.0);
		//pass.setFilterLimits (0.0, 1.5);
		//pass.setFilterLimits (0.0, 0.6);
		pass.setKeepOrganized(false);
		pass.setInputCloud(cloud);
		pass.filter(result);
		FPS_CALC_END("filterPassThrough");
	}

	void euclideanSegment(const CloudConstPtr &cloud,
			std::vector<pcl::PointIndices> &cluster_indices) {
		FPS_CALC_BEGIN;
		pcl::EuclideanClusterExtraction<PointType> ec;
		KdTreePtr tree(new KdTree());

		ec.setClusterTolerance(0.05); // 2cm

		ec.setMinClusterSize(50);

		ec.setMaxClusterSize(25000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);
		ec.extract(cluster_indices);
		FPS_CALC_END("euclideanSegmentation");
	}

	void gridSample(const CloudConstPtr &cloud, Cloud &result,
			double leaf_size = 0.01) {
		FPS_CALC_BEGIN;
		double start = pcl::getTime();
		pcl::VoxelGrid<PointType> grid;
		//pcl::ApproximateVoxelGrid<PointType> grid;
		grid.setLeafSize(leaf_size, leaf_size, leaf_size);
		grid.setInputCloud(cloud);
		grid.filter(result);
		//result = *cloud;
		double end = pcl::getTime();
		downsampling_time_ = end - start;
		FPS_CALC_END("gridSample");
	}

	void gridSampleApprox(const CloudConstPtr &cloud, Cloud &result,
			double leaf_size = 0.01) {
		FPS_CALC_BEGIN;
		double start = pcl::getTime();
		//pcl::VoxelGrid<PointType> grid;
		pcl::ApproximateVoxelGrid<PointType> grid;
		grid.setLeafSize(leaf_size, leaf_size, leaf_size);
		grid.setInputCloud(cloud);
		grid.filter(result);
		//result = *cloud;
		double end = pcl::getTime();
		downsampling_time_ = end - start;
		FPS_CALC_END("gridSample");
	}

	void planeSegmentation(const CloudConstPtr &cloud,
			pcl::ModelCoefficients &coefficients, pcl::PointIndices &inliers) {
		FPS_CALC_BEGIN;
		pcl::SACSegmentation<PointType> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(1000);
		seg.setDistanceThreshold(0.03);
		seg.setInputCloud(cloud);
		seg.segment(inliers, coefficients);
		FPS_CALC_END("planeSegmentation");
	}

	void planeProjection(const CloudConstPtr &cloud, Cloud &result,
			const pcl::ModelCoefficients::ConstPtr &coefficients) {
		FPS_CALC_BEGIN;
		pcl::ProjectInliers<PointType> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(cloud);
		proj.setModelCoefficients(coefficients);
		proj.filter(result);
		FPS_CALC_END("planeProjection");
	}

	void convexHull(const CloudConstPtr &cloud, Cloud &result,
			std::vector<pcl::Vertices> &hull_vertices) {
		FPS_CALC_BEGIN;
		pcl::ConvexHull<PointType> chull;
		chull.setInputCloud(cloud);
		chull.reconstruct(*cloud_hull_, hull_vertices);
		FPS_CALC_END("convexHull");
	}

	void normalEstimation(const CloudConstPtr &cloud,
			pcl::PointCloud<pcl::Normal> &result) {
		FPS_CALC_BEGIN;
		ne_.setInputCloud(cloud);
		ne_.compute(result);
		FPS_CALC_END("normalEstimation");
	}

	void tracking(const RefCloudConstPtr &cloud) {
		double start = pcl::getTime();
		FPS_CALC_BEGIN;
		for (uint track=0; track < tracker_vector_.size(); track++) {

		tracker_vector_[track]->setInputCloud(cloud);
		tracker_vector_[track]->compute();

//		tracker_->setInputCloud(cloud);
//		tracker_->compute();
		}
		double end = pcl::getTime();
		FPS_CALC_END("tracking");
		tracking_time_ = end - start;

	}

	void addNormalToCloud(const CloudConstPtr &cloud,
			const pcl::PointCloud<pcl::Normal>::ConstPtr &normals,
			RefCloud &result) {
		result.width = cloud->width;
		result.height = cloud->height;
		result.is_dense = cloud->is_dense;
		for (size_t i = 0; i < cloud->points.size(); i++) {
			RefPointType point;
			point.x = cloud->points[i].x;
			point.y = cloud->points[i].y;
			point.z = cloud->points[i].z;
			point.rgba = cloud->points[i].rgba;
			// point.normal[0] = normals->points[i].normal[0];
			// point.normal[1] = normals->points[i].normal[1];
			// point.normal[2] = normals->points[i].normal[2];
			result.points.push_back(point);
		}
	}

	void extractNonPlanePoints(const CloudConstPtr &cloud,
			const CloudConstPtr &cloud_hull, Cloud &result) {
		pcl::ExtractPolygonalPrismData<PointType> polygon_extract;
		pcl::PointIndices::Ptr inliers_polygon(new pcl::PointIndices());
		polygon_extract.setHeightLimits(0.01, 10.0);
		polygon_extract.setInputPlanarHull(cloud_hull);
		polygon_extract.setInputCloud(cloud);
		polygon_extract.segment(*inliers_polygon);
		{
			pcl::ExtractIndices<PointType> extract_positive;
			extract_positive.setNegative(false);
			extract_positive.setInputCloud(cloud);
			extract_positive.setIndices(inliers_polygon);
			extract_positive.filter(result);
		}
	}

	void removeZeroPoints(const CloudConstPtr &cloud, Cloud &result) {
		for (size_t i = 0; i < cloud->points.size(); i++) {
			PointType point = cloud->points[i];
			if (!(fabs(point.x) < 0.01 && fabs(point.y) < 0.01
					&& fabs(point.z) < 0.01) && !pcl_isnan(point.x)
					&& !pcl_isnan(point.y) && !pcl_isnan(point.z))
				result.points.push_back(point);
		}

		result.width = result.points.size();
		result.height = 1;
		result.is_dense = true;
	}

	void extractSegmentCluster(const CloudConstPtr &cloud,
			const std::vector<pcl::PointIndices> cluster_indices,
			const int segment_index, Cloud &result) {
		pcl::PointIndices segmented_indices = cluster_indices[segment_index];
		for (size_t i = 0; i < segmented_indices.indices.size(); i++) {
			PointType point = cloud->points[segmented_indices.indices[i]];
			result.points.push_back(point);
		}
		result.width = result.points.size();
		result.height = 1;
		result.is_dense = true;
	}

	void extractLinesNew(const CloudConstPtr &cloud, Cloud &result) {
		std::vector<int> inliers;


		typename pcl::SampleConsensusModelLine<PointType>::Ptr model_l(
				new pcl::SampleConsensusModelLine<PointType>(cloud));

		pcl::RandomSampleConsensus<PointType> ransac(model_l);
		ransac.setDistanceThreshold(.002);
		ransac.computeModel();
		ransac.getInliers(inliers);

		// copies all inliers of the model computed to another PointCloud
		pcl::copyPointCloud<PointType>(*cloud, inliers, result);




	}

	bool extractLines(const CloudConstPtr &cloud, Cloud &result, Cloud &newCloud, pcl::ModelCoefficients::Ptr &coefficients) {

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		if (cloud->size()==0)
			return false;

		pcl::SACSegmentation<PointType> seg;
		// Optional
		seg.setOptimizeCoefficients(true);

		seg.setModelType(pcl::SACMODEL_LINE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.002);

		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0) {
			PCL_ERROR(
					"Could not estimate a line model for the given dataset.");
			return false;
		}
//DEBUG
		/*std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
		 for (size_t i = 0; i < inliers->indices.size(); ++i)
		 std::cerr << inliers->indices[i] << "    "
		 << cloud->points[inliers->indices[i]].x << " "
		 << cloud->points[inliers->indices[i]].y << " "
		 << cloud->points[inliers->indices[i]].z << std::endl;*/

		pcl::copyPointCloud<PointType>(*cloud,newCloud);


		for (size_t i = 0; i < inliers->indices.size(); i++) {
			PointType point = cloud->points[inliers->indices[i]];
			result.points.push_back(point);

		}



		result.width = result.points.size();
		result.height = 1;
		result.is_dense = true;

		//removing line

		RefCloudPtr thickResult(new RefCloud);

		//DEBUG
//		std::cerr<<"result size: "<<result.points.size()<<std::endl;
//
//		std::cerr<<"boundary size: "<<cloud->points.size()<<std::endl;


		extractNeighbor(cloud,result,*thickResult);

		//DEBUG
//		std::cerr<<"thick result size: "<<thickResult->points.size()<<std::endl;

		for (size_t i = 0; i < thickResult->points.size(); i++) {
					PointType point = thickResult->points[i];


					for(size_t j = 0; j < newCloud.points.size(); j++){

						PointType pointNew = newCloud.points[j];

						float dist= sqrt((point.x-pointNew.x)*(point.x-pointNew.x)+(point.y-pointNew.y)*(point.y-pointNew.y)+(point.z-pointNew.z)*(point.z-pointNew.z));
						if(dist<0.00001)
							newCloud.erase(newCloud.begin()+j);



					}




				}

		//DEBUG
		//std::cerr<<"new cloud size: "<<newCloud.points.size()<<std::endl;

		newCloud.width = newCloud.points.size();
		newCloud.height = 1;
		newCloud.is_dense = true;
		return true;

	}

	void findBoundaries(const CloudConstPtr &cloud, Cloud &result) {

		pcl::PointCloud<pcl::Normal>::Ptr normals(
				new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::Normal>::Ptr normals_cloud(
				new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<PointType, pcl::Normal> norm_est;
		norm_est.setSearchMethod(KdTreePtr(new KdTree));
		norm_est.setInputCloud(cloud);
		norm_est.setRadiusSearch(0.03);
		norm_est.compute(*normals_cloud);

		std::cerr<<"cloud before: "<<cloud->size()<<std::endl;


//		 std::sort(normals_cloud->points.begin(),normals_cloud->points.end(),comparison_curv);


		for (size_t i = 0; i < normals_cloud->size(); ++i)
		{
			//std::cerr<<"CURVATURE: "<<normals_cloud->points[i].curvature<<std::endl;
			if (normals_cloud->points[i].curvature > 0.055)
				result.push_back(cloud->points.at(i));
		}

		std::cerr<<"cloud after: "<<result.size()<<std::endl;


		/*

		pcl::PointCloud<pcl::Boundary> boundaries;
		pcl::BoundaryEstimation<PointType, pcl::Normal, pcl::Boundary> est;
		est.setInputCloud(cloud);
		est.setInputNormals(normals_cloud);
		est.setRadiusSearch(0.02); // 2cm radius
		est.setSearchMethod(KdTreePtr(new KdTree));
		est.compute(boundaries);

		for (int i = 0; i < boundaries.size(); i++) {
			if (boundaries.points[i].boundary_point != 0) {
				result.push_back(cloud->points.at(i));
			}
		}
		*/

		result.width = result.points.size();
		result.height = 1;
		result.is_dense = true;

	}

	void extractNeighbor(const CloudConstPtr &cloud,Cloud &searchCloud, Cloud &result){

		  pcl::KdTreeFLANN<PointType> kdtree;

		  kdtree.setInputCloud (cloud);

		  PointType searchPoint;

		  std::vector<int> pointIdxRadiusSearch;
		  std::vector<float> pointRadiusSquaredDistance;

		   float radius = 0.05;




		   for (size_t i = 0; i < searchCloud.size(); ++i){
			   searchPoint=searchCloud.points.at(i);

	   		   	  result.push_back(searchPoint);



			   	   if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
			   	   	   {
			   		   	   for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)

			   		   		   	  result.push_back(cloud->points[ pointIdxRadiusSearch[i] ]);

			   		   	   //DEBUG
			   		   	   /*std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
		                 << " " << cloud->points[ pointIdxRadiusSearch[i] ].y
		                 << " " << cloud->points[ pointIdxRadiusSearch[i] ].z
		                 << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;*/
			   	   	   	 }


		   	 }

		   CloudPtr resultPtr(new Cloud(result));

		   pcl::VoxelGrid<PointType> sor;
		   sor.setInputCloud (resultPtr);
		   sor.setLeafSize (0.001f, 0.001f, 0.001f);
		   sor.filter (result);


		   result.width = result.points.size();
		   		 		result.height = 1;
		   		 		result.is_dense = true;


	}





	bool extractCorners(const CloudConstPtr &cloud, Cloud &result,Cloud &newCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity,int number) {



		 pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>* harris3D = new pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI> (pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>::HARRIS);
		 harris3D->setNonMaxSupression(true);
//		 harris3D->setThreshold(0.0009);
//		 harris3D->setThreshold(0.00011);


		 harris3D->setRadius (0.01);
		 harris3D->setRadiusSearch (0.01);
		 harris3D->setInputCloud(cloud);
		 harris3D->setRefine(false);
		 harris3D->setMethod(pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>::HARRIS);
		 harris3D->compute(*cloud_intensity);

		 pcl::copyPointCloud<PointType>(*cloud,newCloud);


		 cloud_intensity_.reset(new pcl::PointCloud<pcl::PointXYZI>);
		 pcl::copyPointCloud(*cloud_intensity, *cloud_intensity_);

//		 get the best corner

		 bool best_corner=true;

		 if(cloud_intensity_->size()>0){
		 std::sort(cloud_intensity_->points.begin(),cloud_intensity_->points.end(),comparison);

		 if (best_corner)
		 {
		 pcl::PointXYZI max=cloud_intensity_->points[number];
		 pcl::PointXYZI max2=cloud_intensity_->points[number+1];


		 	/* for (size_t i = 0; i < cloud_intensity_->size(); ++i)
		 	 {
		 		   if (max.intensity < cloud_intensity_->points[i].intensity) {
		 			   max = cloud_intensity_->points[i];
		 		   }



		 	 }*/






			 cloud_intensity_.reset(new pcl::PointCloud<pcl::PointXYZI>);
		 	 cloud_intensity_->push_back(max);
		 	 cloud_intensity_->push_back(max2);




		 }


		 pcl::copyPointCloud(*cloud_intensity_, result);


		 pcl::KdTreeFLANN<PointType> kdtree;

				  kdtree.setInputCloud (cloud);

				  PointType searchPoint,searchPoint2;

				  std::vector<int> pointIdxRadiusSearch;
				  std::vector<float> pointRadiusSquaredDistance;

				   float radius = 0.05;
				   int big1,big2;

				   searchPoint=result.points[0];
				   searchPoint2=result.points[1];


				   if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
					   big1=(int)pointIdxRadiusSearch.size ();
				   }

				   pointIdxRadiusSearch.clear();
				   pointRadiusSquaredDistance.clear();

				   if ( kdtree.radiusSearch (searchPoint2, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
				   					   big2=(int)pointIdxRadiusSearch.size ();
				   				   }


				   if (big1>big2)
					   result.erase(result.begin() + 0);
				   else
					   result.erase(result.begin() + 1);


				   RefCloudPtr thickResult(new RefCloud);

				   		//DEBUG
				   //		std::cerr<<"result size: "<<result.points.size()<<std::endl;
				   //
				   //		std::cerr<<"boundary size: "<<cloud->points.size()<<std::endl;


				   		extractNeighbor(cloud,result,*thickResult);

				   		//DEBUG
				   //		std::cerr<<"thick result size: "<<thickResult->points.size()<<std::endl;

				   		for (size_t i = 0; i < thickResult->points.size(); i++) {
				   					PointType point = thickResult->points[i];


				   for(size_t j = 0; j < newCloud.points.size(); j++){

				   						PointType pointNew = newCloud.points[j];

//				   						PointType point =result.points[0];

				   						float dist= sqrt((point.x-pointNew.x)*(point.x-pointNew.x)+(point.y-pointNew.y)*(point.y-pointNew.y)+(point.z-pointNew.z)*(point.z-pointNew.z));
				   						if(dist<0.00001)
				   							newCloud.erase(newCloud.begin()+j);



				   					}
				   		}

		 }
		 else{
			 return false;
		 }



		 		result.width = result.points.size();
		 		result.height = 1;
		 		result.is_dense = true;

		 		return true;


	}

	bool extractPlane(const CloudConstPtr &cloud, Cloud &result,pcl::PointIndices::Ptr &inliers){

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

				if (cloud->size()==0)
					return false;

				pcl::SACSegmentation<PointType> seg;
				// Optional
				seg.setOptimizeCoefficients(true);

				seg.setModelType(pcl::SACMODEL_PLANE);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setDistanceThreshold(0.005);

				seg.setInputCloud(cloud);
				seg.segment(*inliers, *coefficients);
				if (inliers->indices.size() < 60) {
					PCL_ERROR(
							"Could not estimate a line model for the given dataset.");
					return false;
				}
		//DEBUG
				/*std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
				 for (size_t i = 0; i < inliers->indices.size(); ++i)
				 std::cerr << inliers->indices[i] << "    "
				 << cloud->points[inliers->indices[i]].x << " "
				 << cloud->points[inliers->indices[i]].y << " "
				 << cloud->points[inliers->indices[i]].z << std::endl;*/



				for (size_t i = 0; i < inliers->indices.size(); i++) {
					PointType point = cloud->points[inliers->indices[i]];
					result.points.push_back(point);

				}



				result.width = result.points.size();
				result.height = 1;
				result.is_dense = true;

				return true;

	}


	void covariancePCA(const CloudConstPtr &cloud, Eigen::Vector3f &direction){

			Eigen::Vector4f centroid;
			//DEBUG
			//std::cerr << "centroid: " << centroid << std::endl;

				EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
				EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
				Eigen::Matrix3f cov;
				Eigen::Vector3f eigen_vector1;
				Eigen::Vector3f eigen_vector2;
				Eigen::Vector3f vector3rd;


				pcl::compute3DCentroid(*cloud, centroid);


				//orientation of the gripper based on PCA

				pcl::computeCovarianceMatrixNormalized(*cloud, centroid, cov);
				pcl::eigen33(cov, eigen_vectors, eigen_values);


				eigen_vector1(0) = eigen_vectors(0, 2);
				eigen_vector1(1) = eigen_vectors(1, 2);
				eigen_vector1(2) = eigen_vectors(2, 2);
				eigen_vector2(0) = eigen_vectors(0, 1);
				eigen_vector2(1) = eigen_vectors(1, 1);
				eigen_vector2(2) = eigen_vectors(2, 1);

				direction=eigen_vector1;


	}

	void onlyLineNeighbor(const CloudConstPtr &cloud, Cloud &result,Cloud &line, pcl::ModelCoefficients::Ptr &coefficients){

		  CloudPtr cloud_projected (new Cloud);
		  PointType linePoint,linePoint2;

		pcl::ProjectInliers<PointType> proj;
		  proj.setModelType (pcl::SACMODEL_LINE);
		  proj.setInputCloud (cloud);
		  proj.setModelCoefficients (coefficients);
		  proj.filter (*cloud_projected);

		  std::cerr<<"CLOUD SIZE: "<<cloud->size()<<std::endl;

			Eigen::Vector4f c;
			pcl::compute3DCentroid<RefPointType>(line, c);
			float dist,dist2;
			float max_dist=0;
			float max_dist2=0;
			float max_dist_general=0;

			PointType borderPoint1=line.points.at(1);
			PointType borderPoint2=line.points.at(1);
			PointType cloudPoint;

			Cloud resultBefore;

			pcl::copyPointCloud(*cloud, resultBefore);
			  std::cerr<<"CLOUD PROJECTED SIZE: "<<resultBefore.size()<<std::endl;



		  for (size_t i = 0; i < line.size(); ++i){
		  			   linePoint=line.points.at(i);
		  			   dist=sqrt((linePoint.x-c[0])*(linePoint.x-c[0])+(linePoint.y-c[1])*(linePoint.y-c[1])+(linePoint.z-c[2])*(linePoint.z-c[2]));
		  			   if (dist>max_dist){
		  				   max_dist=dist;
		  				   borderPoint1=linePoint;
		  			   }

		  }

		  for (size_t i = 0; i < line.size(); ++i){
		  		  			   linePoint2=line.points.at(i);
		  		  			   dist2=sqrt((linePoint2.x-borderPoint1.x)*(linePoint2.x-borderPoint1.x)+(linePoint2.y-borderPoint1.y)*(linePoint2.y-borderPoint1.y)+(linePoint2.z-borderPoint1.z)*(linePoint2.z-borderPoint1.z));
		  		  			   if (dist2>max_dist2){
		  		  				   max_dist2=dist2;
		  		  				   borderPoint2=linePoint2;
		  		  			   }

		  		  }


			  max_dist_general=/*0.9**/sqrt((borderPoint1.x-borderPoint2.x)*(borderPoint1.x-borderPoint2.x)+(borderPoint1.y-borderPoint2.y)*(borderPoint1.y-borderPoint2.y)+(borderPoint1.z-borderPoint2.z)*(borderPoint1.z-borderPoint2.z));


		  for (size_t i = 0; i < cloud_projected->size(); i++){
			  cloudPoint=cloud_projected->points.at(i);
			  float distance=sqrt((cloudPoint.x-borderPoint1.x)*(cloudPoint.x-borderPoint1.x)+(cloudPoint.y-borderPoint1.y)*(cloudPoint.y-borderPoint1.y)+(cloudPoint.z-borderPoint1.z)*(cloudPoint.z-borderPoint1.z));
			  float distance2=sqrt((cloudPoint.x-borderPoint2.x)*(cloudPoint.x-borderPoint2.x)+(cloudPoint.y-borderPoint2.y)*(cloudPoint.y-borderPoint2.y)+(cloudPoint.z-borderPoint2.z)*(cloudPoint.z-borderPoint2.z));

			  if((distance>max_dist_general)||(distance2>max_dist_general))
			  {
				  resultBefore.points.at(i).x=0;
				  resultBefore.points.at(i).y=0;
				  resultBefore.points.at(i).z=0;
			  }

		  }

		  for (size_t i = 1; i < resultBefore.points.size()+1; i++) {
		  		if ((resultBefore.points[i-1].x == 0)||(resultBefore.points[i-1].y == 0)||(resultBefore.points[i-1].z == 0)) {
		  			resultBefore.erase(resultBefore.begin() + i-1);
		  			resultBefore.width--;
		  			resultBefore.points.resize(resultBefore.width);
		  			i--;
		  		}
		  	}


//		  pcl::copyPointCloud(resultBefore, result);


		  CloudPtr cloudForEuclidianDistance(new Cloud);
		  pcl::copyPointCloud(resultBefore, *cloudForEuclidianDistance);


		  KdTreePtr tree(new KdTree());
		  tree->setInputCloud (cloudForEuclidianDistance);


		  std::vector<pcl::PointIndices> cluster_indices;
		  pcl::EuclideanClusterExtraction<PointType> ec;
		  ec.setClusterTolerance (0.005); // 2cm
		  ec.setMinClusterSize (30);
		  ec.setMaxClusterSize (25000);
		  ec.setSearchMethod (tree);
		  ec.setInputCloud (cloudForEuclidianDistance);
		  ec.extract (cluster_indices);
		  std::cerr<<"Size of result after line euclidian clustering: "<<cloudForEuclidianDistance->points.size()<<std::endl;

		  result.clear();
		  for (size_t i = 0; i < cluster_indices[0].indices.size(); i++) {
		  					PointType point = cloudForEuclidianDistance->points[cluster_indices[0].indices[i]];
		  					result.points.push_back(point);
		  				}


		  std::cerr<<"Size of result after line euclidian clustering: "<<result.points.size()<<std::endl;




		  /*for (size_t i = 0; i < result.size(); i++){
		  if (result.points.at(i).x=0)||(result.points.at(i).y=0)||(result.points.at(i).z=0)

		  }*/

			//pcl::copyPointCloud(*cloud_projected, result);

		 // result.push_back(borderPoint1);
		  //result.push_back(borderPoint2);

		  result.width = result.points.size();
		  result.height = 1;
		  result.is_dense = true;



	}

	int countPlanes(const CloudConstPtr &cloud,Cloud &result){
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		int number=0;


		CloudPtr cloud_plane(new Cloud);
		CloudPtr nonplane_cloud(new Cloud);

		if(extractPlane(cloud,*cloud_plane,inliers)){

	    pcl::ExtractIndices<PointType> extract;
	    extract.setInputCloud (cloud);
	    extract.setIndices (inliers);
	    extract.setNegative (true);

	    // Write the planar inliers to disk
	    extract.filter (*nonplane_cloud);
	    number++;
		}

		cloud_plane.reset(new Cloud);
		inliers.reset(new pcl::PointIndices);
		CloudPtr nonplane_cloud2(new Cloud);

		if(extractPlane(nonplane_cloud,*cloud_plane,inliers)){

			    pcl::ExtractIndices<PointType> extract;
			    extract.setInputCloud (nonplane_cloud);
			    extract.setIndices (inliers);
			    extract.setNegative (true);

			    // Write the planar inliers to disk
			    extract.filter (*nonplane_cloud2);
			    number++;
				}



		cloud_plane.reset(new Cloud);
		inliers.reset(new pcl::PointIndices);
		CloudPtr nonplane_cloud3(new Cloud);

		if(extractPlane(nonplane_cloud2,*cloud_plane,inliers)){

			    pcl::ExtractIndices<PointType> extract;
			    extract.setInputCloud (nonplane_cloud2);
			    extract.setIndices (inliers);
			    extract.setNegative (true);

			    // Write the planar inliers to disk
			    extract.filter (*nonplane_cloud3);
			    number++;
				}

//		pcl::copyPointCloud(*nonplane_cloud,result);



		cloud_plane.reset(new Cloud);
		inliers.reset(new pcl::PointIndices);
		CloudPtr nonplane_cloud4(new Cloud);

		if(extractPlane(nonplane_cloud3,*cloud_plane,inliers)){

			    pcl::ExtractIndices<PointType> extract;
			    extract.setInputCloud (nonplane_cloud3);
			    extract.setIndices (inliers);
			    extract.setNegative (true);

			    // Write the planar inliers to disk
			    extract.filter (*nonplane_cloud4);
			    number++;
				}


		cloud_plane.reset(new Cloud);
		inliers.reset(new pcl::PointIndices);
		CloudPtr nonplane_cloud5(new Cloud);

		if(extractPlane(nonplane_cloud4,*cloud_plane,inliers)){

			    pcl::ExtractIndices<PointType> extract;
			    extract.setInputCloud (nonplane_cloud4);
			    extract.setIndices (inliers);
			    extract.setNegative (true);

			    // Write the planar inliers to disk
			    extract.filter (*nonplane_cloud5);
			    number++;
				}




		std::cerr<<"number of planes: "<<number<<std::endl;



		return number;

	}

	void cloud_cb(const CloudConstPtr &cloud) {
		boost::mutex::scoped_lock lock(mtx_);
		double start = pcl::getTime();
		FPS_CALC_BEGIN;
		bool online = true;

		if (online)
		cloud_pass_.reset(new Cloud);

		cloud_pass_downsampled_.reset(new Cloud);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());




		if(!online){

			if (counter_==0){

			cloud_pass_.reset(new Cloud);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud23 (new pcl::PointCloud<pcl::PointXYZRGB>);
		//CloudPtr cloud23(new CloudPtr);

		  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("cloud1.pcd", *cloud23) == -1) //* load the file
		  {
		    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		  }

		  CloudPtr cloud22(new Cloud);

		  pcl::copyPointCloud(*cloud23, *cloud22);

		  filterPassThrough(cloud22, *cloud_pass_);


			}
		}







		if (online)
		filterPassThrough(cloud, *cloud_pass_);
		if (counter_ < 10) {
			gridSample(cloud_pass_, *cloud_pass_downsampled_,
					downsampling_grid_size_);
		} else if (counter_ == 10) {
			//gridSample (cloud_pass_, *cloud_pass_downsampled_, 0.01);
			cloud_pass_downsampled_ = cloud_pass_;
			CloudPtr target_cloud;
			if (use_convex_hull_) {
				planeSegmentation(cloud_pass_downsampled_, *coefficients,
						*inliers);
				if (inliers->indices.size() > 3) {
					CloudPtr cloud_projected(new Cloud);
					cloud_hull_.reset(new Cloud);
					nonplane_cloud_.reset(new Cloud);

					planeProjection(cloud_pass_downsampled_, *cloud_projected,
							coefficients);
					convexHull(cloud_projected, *cloud_hull_, hull_vertices_);

					extractNonPlanePoints(cloud_pass_downsampled_, cloud_hull_,
							*nonplane_cloud_);
					target_cloud = nonplane_cloud_;
				} else {
					PCL_WARN("cannot segment plane\n");
				}
			} else {
				PCL_WARN("without plane segmentation\n");
				target_cloud = cloud_pass_downsampled_;
			}






			if (target_cloud != NULL) {
				PCL_INFO("segmentation, please wait...\n");
				std::vector<pcl::PointIndices> cluster_indices;
				euclideanSegment(target_cloud, cluster_indices);
				if (cluster_indices.size() > 0) {
					// select the cluster to track
					CloudPtr temp_cloud(new Cloud);
					extractSegmentCluster(target_cloud, cluster_indices, 0,
							*temp_cloud);
					Eigen::Vector4f c;
					pcl::compute3DCentroid<RefPointType>(*temp_cloud, c);
					int segment_index = 0;
					double segment_distance = c[0] * c[0] + c[1] * c[1];
					for (size_t i = 1; i < cluster_indices.size(); i++) {
						temp_cloud.reset(new Cloud);
						extractSegmentCluster(target_cloud, cluster_indices, i,
								*temp_cloud);
						pcl::compute3DCentroid<RefPointType>(*temp_cloud, c);
						double distance = c[0] * c[0] + c[1] * c[1];
						if (distance < segment_distance) {
							segment_index = i;
							segment_distance = distance;
						}
					}

					segmented_cloud_.reset(new Cloud);
					extractSegmentCluster(target_cloud, cluster_indices,
							segment_index, *segmented_cloud_);
					//pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
					//normalEstimation (segmented_cloud_, *normals);
					RefCloudPtr ref_cloud(new RefCloud);
					//addNormalToCloud (segmented_cloud_, normals, *ref_cloud);
					ref_cloud = segmented_cloud_;
					RefCloudPtr nonzero_ref(new RefCloud);
					removeZeroPoints(ref_cloud, *nonzero_ref);







					RefCloudPtr nonzero_ref_lines(new RefCloud);
					RefCloudPtr nonzero_ref_lines2(new RefCloud);
					RefCloudPtr nonzero_ref_lines3(new RefCloud);


					RefCloudPtr nonzero_ref_boundary(new RefCloud);
					RefCloudPtr nonzero_ref_corners(new RefCloud);
					RefCloudPtr nonzero_ref_corners2(new RefCloud);

					RefCloudPtr nonzero_ref_plane(new RefCloud);


					RefCloudPtr nonzero_ref_no_corner(new RefCloud);
					RefCloudPtr nonzero_ref_no_corner2(new RefCloud);



					RefCloudPtr nonzero_ref_no_line(new RefCloud);
					RefCloudPtr nonzero_ref_no_line2(new RefCloud);
					RefCloudPtr nonzero_ref_no_line3(new RefCloud);

					pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
					pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients);
					pcl::ModelCoefficients::Ptr coefficients3(new pcl::ModelCoefficients);
					pcl::ModelCoefficients::Ptr coefficients4(new pcl::ModelCoefficients);

					Eigen::Vector3f direction1;
					Eigen::Vector3f direction2;
					Eigen::Vector3f direction3;
					Eigen::Vector3f direction4;



					std::vector<RefCloudPtr> clouds_vector;
					std::vector<bool> is_line_vector;
					std::vector<pcl::ModelCoefficients::Ptr> coeff_vector;
					std::vector<Eigen::Vector3f> directions_vector;



					pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity(new pcl::PointCloud<pcl::PointXYZI>);



					//finding boundaries
					findBoundaries(nonzero_ref, *nonzero_ref_boundary);
					//extract the corners

					if(extractCorners(nonzero_ref, *nonzero_ref_corners,*nonzero_ref_no_corner,cloud_intensity,0)){

						clouds_vector.push_back(nonzero_ref_corners);
						is_line_vector.push_back(false);
						coeff_vector.push_back(coefficients4);
						directions_vector.push_back(direction1);
					}
////
					if(extractCorners(nonzero_ref_no_corner, *nonzero_ref_corners2,*nonzero_ref_no_corner2,cloud_intensity,0)){


						clouds_vector.push_back(nonzero_ref_corners2);
						is_line_vector.push_back(false);
						coeff_vector.push_back(coefficients4);
						directions_vector.push_back(direction2);

					}

					//extracting first line
					if(extractLines(nonzero_ref_boundary, *nonzero_ref_lines,*nonzero_ref_no_line,coefficients)){

						clouds_vector.push_back(nonzero_ref_lines);
						is_line_vector.push_back(true);
						coeff_vector.push_back(coefficients);
						covariancePCA(nonzero_ref_lines,direction3);
						std::cerr<<"DIRECTION"<<direction3<<std::endl;
						directions_vector.push_back(direction3);


					}

					//extracting second line

//					std::cerr<< "true or false: "<< extractLines(nonzero_ref_no_line, *nonzero_ref_lines2,*nonzero_ref_no_line2,coefficients2)<<std::endl;

					if(extractLines(nonzero_ref_no_line, *nonzero_ref_lines2,*nonzero_ref_no_line2,coefficients2)){
											clouds_vector.push_back(nonzero_ref_lines2);
											is_line_vector.push_back(true);
											coeff_vector.push_back(coefficients2);
											covariancePCA(nonzero_ref_lines2,direction4);
											directions_vector.push_back(direction4);

					}


//
//					extractLines(nonzero_ref_no_line2, *nonzero_ref_lines3,*nonzero_ref_no_line3,coefficients3);


//					if(extractPlane(nonzero_ref,*nonzero_ref_plane)){
//										clouds_vector.push_back(nonzero_ref_plane);
//										is_line_vector.push_back(true);
//										coeff_vector.push_back(coefficients2);
//					}


					tracker_vector_.resize(clouds_vector.size());







//					clouds_vector.push_back(nonzero_ref_lines3);
//					is_line_vector.push_back(true);
//					coeff_vector.push_back(coefficients3);







					for (uint track=0; track < tracker_vector_.size(); track++) {

						RefCloudPtr nonzero_ref_final_cloud(new RefCloud);
						RefCloudPtr nonzero_ref_very_final_cloud(new RefCloud);


					//making point cloud thicker- in case of change(i.e. corner instead of the line or so) change second param
					extractNeighbor(nonzero_ref,*clouds_vector[track], *nonzero_ref_final_cloud);



					int planes_number=countPlanes(nonzero_ref_final_cloud,*nonzero_ref_final_cloud);

					if ((is_line_vector[track])&&(coeff_vector[track]!=NULL)){
						if(( planes_number< 2)||(planes_number > 3)){
							std::cerr<<"delete this line"<<std::endl;
							}

					onlyLineNeighbor(nonzero_ref_final_cloud,*nonzero_ref_final_cloud,*clouds_vector[track],coeff_vector[track]);


					std::vector<double> step_covariance = std::vector<double>(6,
									0.005 * 0.015);//0.000075
							step_covariance[3] *= 40.0;
							step_covariance[4] *= 40.0;
							step_covariance[5] *= 40.0;

					directions_vector[track].normalize();
					std::cerr<<"direction vector x: "<<fabs (directions_vector[track][0])<<std::endl;
					std::cerr<<"direction vector y: "<<fabs (directions_vector[track][1])<<std::endl;
					std::cerr<<"direction vector z: "<<fabs (directions_vector[track][2])<<std::endl;


					step_covariance[0]*=fabs (directions_vector[track][0]);
					step_covariance[1]*=fabs (directions_vector[track][1]);
					step_covariance[2]*=fabs (directions_vector[track][2]);




					tracker_vector_[track]->setStepNoiseCovariance(step_covariance);

					}
					else{
					if (planes_number<3){
						std::cerr<<"delete this corner"<<std::endl;
					}
					}
					std::stringstream ss;
						ss <<track;
//					 pcl::io::savePCDFileASCII (ss.str()+".pcd", *nonzero_ref_final_cloud);








					PCL_INFO("calculating cog\n");

					RefCloudPtr transed_ref(new RefCloud);
					//pcl::compute3DCentroid<RefPointType> (*nonzero_ref, c);
					pcl::compute3DCentroid<RefPointType>(*nonzero_ref_final_cloud, c);

					Eigen::Affine3f trans = Eigen::Affine3f::Identity();
					trans.translation() = Eigen::Vector3f(c[0], c[1], c[2]);
					//pcl::transformPointCloudWithNormals<RefPointType> (*ref_cloud, *transed_ref, trans.inverse());
					//pcl::transformPointCloud<RefPointType> (*nonzero_ref, *transed_ref, trans.inverse());
					pcl::transformPointCloud<RefPointType>(*nonzero_ref_final_cloud,
							*transed_ref, trans.inverse());

					CloudPtr transed_ref_downsampled(new Cloud);
					gridSample(transed_ref, *transed_ref_downsampled,
							downsampling_grid_size_);
					tracker_vector_[track]->setReferenceCloud(transed_ref_downsampled);
					tracker_vector_[track]->setTrans(trans);
					reference_ = transed_ref;
					tracker_vector_[track]->setMinIndices(ref_cloud->points.size() / 2);
				}
			}
			else {
					PCL_WARN("euclidean segmentation failed\n");
				}

			}
		} else {
			//normals_.reset (new pcl::PointCloud<pcl::Normal>);
			//normalEstimation (cloud_pass_downsampled_, *normals_);
			//RefCloudPtr tracking_cloud (new RefCloud ());
			//addNormalToCloud (cloud_pass_downsampled_, normals_, *tracking_cloud);
			//tracking_cloud = cloud_pass_downsampled_;

			//*cloud_pass_downsampled_ = *cloud_pass_;
			//cloud_pass_downsampled_ = cloud_pass_;
			gridSampleApprox(cloud_pass_, *cloud_pass_downsampled_,
					downsampling_grid_size_);
			tracking(cloud_pass_downsampled_);
		}

		new_cloud_ = true;
		double end = pcl::getTime();
		computation_time_ = end - start;
		FPS_CALC_END("computation");
		if (cloud_intensity_ !=NULL)
		{
			//DEBUG
		 //std::cerr<<"number of points: "<<cloud_intensity_->size()<<std::endl;
		/*	std::cerr<<"START"<<std::endl;
		 for (size_t i = 0; i < cloud_intensity_->size(); ++i)
		     std::cerr << "    " << cloud_intensity_->points[i].intensity << std::endl;
			std::cerr<<"STOP"<<std::endl;
		 */

		}
		counter_++;

	}

	void run() {
		pcl::Grabber* interface = new pcl::OpenNIGrabber(device_id_);
		boost::function<void(const CloudConstPtr&)> f = boost::bind(
				&OpenNISegmentTracking::cloud_cb, this, _1);
		interface->registerCallback(f);

		viewer_.runOnVisualizationThread(
				boost::bind(&OpenNISegmentTracking::viz_cb, this, _1),
				"viz_cb");

		interface->start();

		while (!viewer_.wasStopped())
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		interface->stop();
	    a_file_.close();

	}

	pcl::visualization::CloudViewer viewer_;
	pcl::PointCloud<pcl::Normal>::Ptr normals_;
	CloudPtr cloud_pass_;
	CloudPtr cloud_pass_downsampled_;
	CloudPtr plane_cloud_;
	CloudPtr nonplane_cloud_;
	CloudPtr cloud_hull_;
	CloudPtr segmented_cloud_;
	CloudPtr reference_;

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity_;

	Eigen::Affine3f transformation_;


	std::vector<pcl::Vertices> hull_vertices_;

	std::string device_id_;
	boost::mutex mtx_;
	bool new_cloud_;
	pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_; // to store threadpool
	boost::shared_ptr<ParticleFilter> tracker_;
	std::vector<boost::shared_ptr<ParticleFilter> > tracker_vector_;
	int counter_;
	bool use_convex_hull_;
	bool visualize_non_downsample_;
	bool visualize_particles_;
	double tracking_time_;
	double computation_time_;
	double downsampling_time_;
	double downsampling_grid_size_;
	std::ofstream a_file_ ;

};



void usage(char** argv) {
	std::cout << "usage: " << argv[0] << " <device_id> [-C] [-g]\n\n";
	std::cout
			<< "  -C:  initialize the pointcloud to track without plane segmentation"
			<< std::endl;
	std::cout << "  -D: visualizing with non-downsampled pointclouds."
			<< std::endl;
	std::cout << "  -P: not visualizing particle cloud." << std::endl;
	std::cout << "  -fixed: use the fixed number of the particles."
			<< std::endl;
	std::cout
			<< "  -d <value>: specify the grid size of downsampling (defaults to 0.01)."
			<< std::endl;
}

int main(int argc, char** argv) {
	bool use_convex_hull = true;
	bool visualize_non_downsample = false;
	bool visualize_particles = true;
	bool use_fixed = false;

	double downsampling_grid_size = 0.01;

	if (pcl::console::find_argument(argc, argv, "-C") > 0)
		use_convex_hull = false;
	if (pcl::console::find_argument(argc, argv, "-D") > 0)
		visualize_non_downsample = true;
	if (pcl::console::find_argument(argc, argv, "-P") > 0)
		visualize_particles = false;
	if (pcl::console::find_argument(argc, argv, "-fixed") > 0)
		use_fixed = true;
	pcl::console::parse_argument(argc, argv, "-d", downsampling_grid_size);
	if (argc < 2) {
		usage(argv);
		exit(1);
	}

	std::string device_id = std::string(argv[1]);

	if (device_id == "--help" || device_id == "-h") {
		usage(argv);
		exit(1);
	}

	// open kinect
	OpenNISegmentTracking<pcl::PointXYZRGBA> v(device_id, 8,
			downsampling_grid_size, use_convex_hull, visualize_non_downsample,
			visualize_particles, use_fixed,4);
	v.run();
}
