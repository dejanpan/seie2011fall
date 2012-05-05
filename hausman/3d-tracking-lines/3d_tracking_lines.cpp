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
			bool use_fixed) :
			viewer_("PCL OpenNI Tracking Viewer"), device_id_(device_id), new_cloud_(
					false), ne_(thread_nr), counter_(0), use_convex_hull_(
					use_convex_hull), visualize_non_downsample_(
					visualize_non_downsample), visualize_particles_(
					visualize_particles), downsampling_grid_size_(
					downsampling_grid_size) {
		KdTreePtr tree(new KdTree(false));
		ne_.setSearchMethod(tree);
		ne_.setRadiusSearch(0.03);
		//std::vector<double> default_step_covariance = std::vector<double>(6,
		//		0.015 * 0.015);
		std::vector<double> default_step_covariance = std::vector<double>(6,
				0.005 * 0.015);
		default_step_covariance[3] *= 40.0;
		default_step_covariance[4] *= 40.0;
		default_step_covariance[5] *= 40.0;

		std::cerr << "step covariance: " << default_step_covariance.size()
				<< std::endl;

		std::vector<double> initial_noise_covariance = std::vector<double>(6,
				0.00001);
		std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);
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
		//mycode
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
	}

	bool drawParticles(pcl::visualization::PCLVisualizer& viz) {
		ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
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
					/*	pcl::visualization::PointCloudColorHandlerCustom<
					 pcl::PointXYZ> blue_color(particle_cloud, 250, 99,
					 71);
					 if (!viz.updatePointCloud(particle_cloud, blue_color,
					 "particle cloud"))
					 viz.addPointCloud(particle_cloud, blue_color,
					 "particle cloud");*/
				}
			}
			return true;
		} else {
			PCL_WARN("no particles\n");
			return false;
		}
	}

	void drawResult(pcl::visualization::PCLVisualizer& viz) {
		ParticleXYZRPY result = tracker_->getResult();
		bool non_tracking=false;

		if (non_tracking)
		{
			if (counter_<13){

				Eigen::Affine3f transformation = tracker_->toEigenMatrix(result);
				// move a little bit for better visualization
				transformation.translation() += Eigen::Vector3f(0.0, 0.0, -0.005);
				transformation_=transformation;
				}
		}
		else
		{
			Eigen::Affine3f transformation = tracker_->toEigenMatrix(result);
							// move a little bit for better visualization
							transformation.translation() += Eigen::Vector3f(0.0, 0.0, -0.005);
							transformation_=transformation;
		}


		RefCloudPtr result_cloud(new RefCloud());

		if (!visualize_non_downsample_)
			pcl::transformPointCloud<RefPointType>(
					*(tracker_->getReferenceCloud()), *result_cloud,
					transformation_);
		else
			pcl::transformPointCloud<RefPointType>(*reference_, *result_cloud,
					transformation_);

		{
			pcl::visualization::PointCloudColorHandlerCustom<RefPointType> red_color(
					result_cloud, 0, 0, 255);
			if (!viz.updatePointCloud(result_cloud, red_color, "resultcloud"))
				viz.addPointCloud(result_cloud, red_color, "resultcloud");


			 viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "resultcloud");
			 //pcl::PointXYZ o;
			   // o.x = 1.0;
			    //o.y = 0;
			    //o.z = 0;
			   // viz.addSphere (o, 0.25, "sphere", 0);
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
				viz.removeShape("N");
				viz.addText(
						(boost::format("number of Reference PointClouds: %d")
								% tracker_->getReferenceCloud()->points.size()).str(),
						10, 20, 20, 1.0, 1.0, 1.0, "N");

				viz.removeShape("M");
				viz.addText(
						(boost::format("number of Measured PointClouds:  %d")
								% cloud_pass_downsampled_->points.size()).str(),
						10, 40, 20, 1.0, 1.0, 1.0, "M");

				viz.removeShape("tracking");
				viz.addText(
						(boost::format("tracking:        %f fps")
								% (1.0 / tracking_time_)).str(), 10, 60, 20,
						1.0, 1.0, 1.0, "tracking");

				viz.removeShape("downsampling");
				viz.addText(
						(boost::format("downsampling:    %f fps")
								% (1.0 / downsampling_time_)).str(), 10, 80, 20,
						1.0, 1.0, 1.0, "downsampling");

				viz.removeShape("computation");
				viz.addText(
						(boost::format("computation:     %f fps")
								% (1.0 / computation_time_)).str(), 10, 100, 20,
						1.0, 1.0, 1.0, "computation");

				viz.removeShape("particles");
				viz.addText(
						(boost::format("particles:     %d")
								% tracker_->getParticles()->points.size()).str(),
						10, 120, 20, 1.0, 1.0, 1.0, "particles");

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
		tracker_->setInputCloud(cloud);
		tracker_->compute();
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

	void extractLines(const CloudConstPtr &cloud, Cloud &result, Cloud &newCloud) {

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		pcl::SACSegmentation<PointType> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory

		seg.setModelType(pcl::SACMODEL_LINE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.002);

		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0) {
			PCL_ERROR(
					"Could not estimate a line model for the given dataset.");
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
			//newCloud.erase(newCloud.begin()+i);

		}

		//newCloud.width = newCloud.points.size();
		//newCloud.height = 1;
		//newCloud.is_dense = true;

		result.width = result.points.size();
		result.height = 1;
		result.is_dense = true;

		//removing line

		RefCloudPtr thickResult(new RefCloud);

		std::cerr<<"result size: "<<result.points.size()<<std::endl;

		std::cerr<<"boundary size: "<<cloud->points.size()<<std::endl;


		extractNeighbor(cloud,result,*thickResult);

		std::cerr<<"thick result size: "<<thickResult->points.size()<<std::endl;

		for (size_t i = 0; i < thickResult->points.size(); i++) {
					PointType point = thickResult->points[i];
					//result.points.push_back(point);
					//newCloud.erase(newCloud.begin()+i);

					for(size_t j = 0; j < newCloud.points.size(); j++){

						PointType pointNew = newCloud.points[j];

						float dist= sqrt((point.x-pointNew.x)*(point.x-pointNew.x)+(point.y-pointNew.y)*(point.y-pointNew.y)+(point.z-pointNew.z)*(point.z-pointNew.z));
						if(dist<0.00001)
							newCloud.erase(newCloud.begin()+j);



					}




				}

		std::cerr<<"new cloud size: "<<newCloud.points.size()<<std::endl;

		newCloud.width = newCloud.points.size();
		newCloud.height = 1;
		newCloud.is_dense = true;

	}

	void findBoundaries(const CloudConstPtr &cloud, Cloud &result) {

		pcl::PointCloud<pcl::Normal>::Ptr normals(
				new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::Normal>::Ptr normals_cloud(
				new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<PointType, pcl::Normal> norm_est;
		norm_est.setSearchMethod(KdTreePtr(new KdTree));
		norm_est.setInputCloud(cloud);
		norm_est.setRadiusSearch(0.01);
		norm_est.compute(*normals_cloud);

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

		   float radius = 0.02;




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





	void extractCorners(const CloudConstPtr &cloud, Cloud &result, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity) {



		 pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>* harris3D = new pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI> (pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>::HARRIS);
		 harris3D->setNonMaxSupression(true);
		 harris3D->setThreshold(0.0009);
//		 harris3D->setThreshold(0.00011);


		 harris3D->setRadius (0.01);
		 harris3D->setRadiusSearch (0.01);
		 harris3D->setInputCloud(cloud);
		 harris3D->setRefine(false);
		 harris3D->setMethod(pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>::HARRIS);
		 harris3D->compute(*cloud_intensity);


		 cloud_intensity_.reset(new pcl::PointCloud<pcl::PointXYZI>);
		 pcl::copyPointCloud(*cloud_intensity, *cloud_intensity_);

//		 get the best corner

		 bool best_corner=true;

		 if (best_corner)
		 {
		 pcl::PointXYZI max=cloud_intensity_->points[1];
		 	 for (size_t i = 0; i < cloud_intensity_->size(); ++i)
		 	 {
		 		   if (max.intensity < cloud_intensity_->points[i].intensity) {
		 			   max = cloud_intensity_->points[i];
		 		   }



		 	 }

			 cloud_intensity_.reset(new pcl::PointCloud<pcl::PointXYZI>);
		 	 cloud_intensity_->push_back(max);

		 }


		 pcl::copyPointCloud(*cloud_intensity_, result);


		 		result.width = result.points.size();
		 		result.height = 1;
		 		result.is_dense = true;


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

					RefCloudPtr nonzero_ref_boundary(new RefCloud);
					RefCloudPtr nonzero_ref_corners(new RefCloud);
					RefCloudPtr nonzero_ref_final_cloud(new RefCloud);
					RefCloudPtr nonzero_ref_no_line(new RefCloud);
					RefCloudPtr nonzero_ref_no_line2(new RefCloud);


					pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity(new pcl::PointCloud<pcl::PointXYZI>);

					findBoundaries(nonzero_ref, *nonzero_ref_boundary);
					extractCorners(nonzero_ref, *nonzero_ref_corners,cloud_intensity);
					extractLines(nonzero_ref_boundary, *nonzero_ref_lines,*nonzero_ref_no_line);

					extractLines(nonzero_ref_no_line, *nonzero_ref_lines2,*nonzero_ref_no_line2);



					extractNeighbor(nonzero_ref,*nonzero_ref_lines2, *nonzero_ref_final_cloud);








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
					tracker_->setReferenceCloud(transed_ref_downsampled);
					tracker_->setTrans(trans);
					reference_ = transed_ref;
					tracker_->setMinIndices(ref_cloud->points.size() / 2);
				} else {
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
		 //std::cerr<<"number of points: "<<cloud_intensity_->size()<<std::endl;
			std::cerr<<"START"<<std::endl;
		 for (size_t i = 0; i < cloud_intensity_->size(); ++i)
		     std::cerr << "    " << cloud_intensity_->points[i].intensity << std::endl;
			std::cerr<<"STOP"<<std::endl;


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
	int counter_;
	bool use_convex_hull_;
	bool visualize_non_downsample_;
	bool visualize_particles_;
	double tracking_time_;
	double computation_time_;
	double downsampling_time_;
	double downsampling_grid_size_;
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
			visualize_particles, use_fixed);
	v.run();
}
