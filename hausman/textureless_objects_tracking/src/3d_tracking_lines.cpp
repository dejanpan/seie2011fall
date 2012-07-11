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
#include "PrimitivesExtract.cpp"
#include "textureless_objects_tracking/point_type.h"


#include <ros/ros.h>

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
		//std::vector<double> default_step_covariance = std::vector<double>(6,
		//		0.015 * 0.015);
		std::vector<double> default_step_covariance = std::vector<double>(6,
				0.005 * 0.015);
		default_step_covariance[3] *= 40.0;
		default_step_covariance[4] *= 40.0;
		default_step_covariance[5] *= 40.0;

		step_covariance_ = default_step_covariance;



		std::vector<double> initial_noise_covariance = std::vector<double>(6,
				0.00001);
		std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

		for (int i = 0; i < trackers_number; i++) {

			if (use_fixed) {
				boost::shared_ptr<
						ParticleFilterOMPTracker<RefPointType, ParticleT> > tracker(
						new ParticleFilterOMPTracker<RefPointType, ParticleT>(
								thread_nr));
				tracker_ = tracker;

			} else {
				boost::shared_ptr<
						KLDAdaptiveParticleFilterOMPTracker<RefPointType,
								ParticleT> > tracker(
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


//		tracker_->setParticleNum(400);
			tracker_->setParticleNum(800);

			tracker_->setResampleLikelihoodThr(0.00);
			tracker_->setUseNormal(false);

			tracker_->setMotionRatio(0.2);

			// setup coherences
			ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence =
					ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr(
							new ApproxNearestPairPointCloudCoherence<
									RefPointType>());
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

		unsigned char colormap_[36] = { 255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0,
				255, 0, 255, 0, 255, 255, 127, 0, 0, 0, 127, 0, 0, 0, 127, 127, 127, 0,
				127, 0, 127, 0, 127, 127 };

		for (uint track = 0; track < tracker_vector_.size(); track++) {
			bool drawParticles = true;
			ParticleFilter::PointCloudStatePtr particles =
					tracker_vector_[track]->getParticles();
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
						if (drawParticles) {

							pcl::visualization::PointCloudColorHandlerCustom<
									pcl::PointXYZ> blue_color(particle_cloud,
									colormap_[3 * track],
									colormap_[3 * track + 1],
									colormap_[3 * track + 2]);
//					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color(particle_cloud, 250, 99,71);
							if (!viz.updatePointCloud(particle_cloud,
									blue_color, "particle cloud" + track))
								viz.addPointCloud(particle_cloud, blue_color,
										"particle cloud" + track);
						}
					}
				}

			} else {
				PCL_WARN("no particles\n");
				return false;
			}
		}
		return true;
	}

	void drawResult(pcl::visualization::PCLVisualizer& viz) {

		unsigned char colormap_[36] = { 255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0,
				255, 0, 255, 0, 255, 255, 127, 0, 0, 0, 127, 0, 0, 0, 127, 127, 127, 0,
				127, 0, 127, 0, 127, 127 };

		for (uint track = 0; track < tracker_vector_.size(); track++) {

			ParticleXYZRPY result = tracker_vector_[track]->getResult();
			bool non_tracking = false;
			float row, pitch, yaw, x, y, z;

			if (non_tracking) {
				if (counter_ < 13) {

					Eigen::Affine3f transformation =
							tracker_vector_[track]->toEigenMatrix(result);
					// move a little bit for better visualization
					transformation.translation() += Eigen::Vector3f(0.0, 0.0,
							-0.005);
					transformation_ = transformation;
				}
			} else {
				Eigen::Affine3f transformation =
						tracker_vector_[track]->toEigenMatrix(result);
				transformation.translation() += Eigen::Vector3f(0.0, 0.0,
						-0.005);
				transformation_ = transformation;


			}

			RefCloudPtr result_cloud(new RefCloud());

			if (!visualize_non_downsample_)
				pcl::transformPointCloud<RefPointType>(
						*(tracker_vector_[track]->getReferenceCloud()),
						*result_cloud, transformation_);
			else

				pcl::transformPointCloud<RefPointType>(
						*(tracker_vector_[track]->getReferenceCloud()),
						*result_cloud, transformation_);

			Eigen::Vector4f center;
			pcl::compute3DCentroid<RefPointType>(*result_cloud, center);

			std::stringstream ss;
			ss << track;

			a_file_ << center[0] << " " << center[1] << " " << center[2]
					<< std::endl;


			{

				pcl::visualization::PointCloudColorHandlerCustom<RefPointType> red_color(
						result_cloud, colormap_[3 * track],
						colormap_[3 * track + 1], colormap_[3 * track + 2]);


				if (!viz.updatePointCloud(result_cloud, red_color, ss.str()))
					viz.addPointCloud(result_cloud, red_color, ss.str());



//				if((!viz.updatePointCloud(result_cloud, red_color, ss.str()))&&(!viz.addPointCloud(result_cloud, red_color, ss.str())))
//					viz.removePointCloud()


				//points' size
				viz.setPointCloudRenderingProperties(
						pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,
						ss.str());

			}
		}
	}

	void viz_cb(pcl::visualization::PCLVisualizer& viz) {
		boost::mutex::scoped_lock lock(mtx_);


		viz.setBackgroundColor(255,255,255);
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

		if (viz.wasStopped())
			std::cout<<"Program stopped"<<std::endl;

		if(feature_lost_>-1){
			std::stringstream ss_del;
			ss_del << feature_lost_;
			viz.removePointCloud(ss_del.str());

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

				for (uint track = 0; track < tracker_vector_.size(); track++) {
//						tracker_vector_[track]->getParticles();
					std::stringstream ss;
					ss << track;


					viz.removeShape("particles" + ss.str());
//					viz.addText(
//							(boost::format("particles:     %d")
//									% (tracker_vector_[track]->getParticles()->points.size())).str(),
//							10, 120 * track, 20, 1.0, 1.0, 1.0,
//							"particles" + ss.str());
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

		//25000
		ec.setMaxClusterSize(50000);
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
		for (uint track = 0; track < tracker_vector_.size(); track++) {
			feature_lost_=-1;

			tracker_vector_[track]->setInputCloud(cloud);
			tracker_vector_[track]->compute();
		if((tracker_vector_[track]->getResult().weight<0.0035)&&(counter_>40)){
			std::cout<<"probability of " <<track<<" is : "<<tracker_vector_[track]->getResult().weight<<std::endl;
			std::cout<<"Feature "<<track<<" is lost!"<<std::endl;
			feature_lost_=track;
			tracker_vector_.erase(tracker_vector_.begin()+track);


		}

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

	void cloud_cb(const CloudConstPtr &cloud) {
		boost::mutex::scoped_lock lock(mtx_);
		double start = pcl::getTime();
		FPS_CALC_BEGIN;
		bool online = true;

		if(counter_==0){

	    cv::Mat cvimage(cloud->width,cloud->height, CV_8UC3);
        for (uint h = 0; h < cloud->height; h++) {
                for (uint w = 0; w < cloud->width; w++) {
            //Get colour data for our cvimage
                        cvimage.at<cv::Vec3b>(w, h)[0] = cloud->at(h * cloud->width + w).b;
                        cvimage.at<cv::Vec3b>(w, h)[1] = cloud->at(h * cloud->width + w).g;
                        cvimage.at<cv::Vec3b>(w, h)[2] = cloud->at(h * cloud->width + w).r;

                }
        }
    //Transpose
        cvimage = cvimage.t();
//        textureless_objects_tracking::cornerFind::Response res_corner;
        cv::Mat bw_image(cvimage.rows,cvimage.cols,CV_8U);
        cv::cvtColor(cvimage,bw_image ,CV_BGR2GRAY);
        bw_image_=bw_image;
//            cv::namedWindow( "Display window image", CV_WINDOW_AUTOSIZE );
//            cv::imshow( "Display window image", bw_image );
//            cv::waitKey(0);
//		PrimitivesExtract<pcl::PointXYZRGBA> prim_ex_for_concave;
////
////
//		prim_ex_for_concave.getCornersToPush(bw_image,res_corner);
//		ros::Duration(2.0);
//        std::cout<<"res_corner: "<<res_corner.corner.size()<<std::endl;
//        std::cout<<"res_corner convex: "<<res_corner.corner_convex.size()<<std::endl;
		std::cerr<<"cloud height: "<<cloud->height<<std::endl;
		std::cerr<<"cloud width: "<<cloud->width<<std::endl;

		}
		if (online)
			cloud_pass_.reset(new Cloud);

		cloud_pass_downsampled_.reset(new Cloud);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		if (!online) {

			if (counter_ == 0) {

				cloud_pass_.reset(new Cloud);

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud23(
						new pcl::PointCloud<pcl::PointXYZRGB>);
				//CloudPtr cloud23(new CloudPtr);

				if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("cloud1.pcd",
						*cloud23) == -1) //* load the file
						{
					PCL_ERROR("Couldn't read file test_pcd.pcd \n");
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

					PrimitivesExtract<pcl::PointXYZRGBA> prim_ex(nonzero_ref);
					prim_ex.setPlaneCoefficients(coefficients);
					std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> result_vector_lines;
					std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> result_vector_corners;
					std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> result_vector_cylinders;
					std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> result_vector_circles;
					std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> result_vector;
					std::vector<Eigen::Vector3f> directions_vector;

					std::string what = "rectangular";


					pcl::PointCloud<pcl::PointXYZLRegion>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZLRegion>);
					prim_ex.getSegments(nonzero_ref,cloud_out);
				    pcl::io::savePCDFile("result.pcd",*cloud_out);

				    pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_to_save(new pcl::PointCloud<pcl::PointXYZLRegionF>);

				    pcl::copyPointCloud(*cloud_out,*cloud_to_save);
					RefCloudPtr cloud_to_save_temp(new RefCloud);
				    pcl::copyPointCloud(*cloud_out,*cloud_to_save_temp);




				    pcl::PointCloud<pcl::PointXYZLRegion>::Ptr label_rectangular(new pcl::PointCloud<pcl::PointXYZLRegion>);
				    pcl::PointCloud<pcl::PointXYZLRegion>::Ptr label_circular(new pcl::PointCloud<pcl::PointXYZLRegion>);
				    pcl::PointCloud<pcl::PointXYZLRegion>::Ptr label_other(new pcl::PointCloud<pcl::PointXYZLRegion>);

					RefCloudPtr nonzero_ref_rectangular(new RefCloud);
					RefCloudPtr nonzero_ref_circular(new RefCloud);
					RefCloudPtr nonzero_ref_other(new RefCloud);

					std::vector<uint32_t> vec;


					for (size_t i = 0; i < cloud_out->points.size(); i++) {
						uint32_t label=cloud_out->points[i].label;

						if((label==1)||(label==4)){
							label_circular->push_back(cloud_out->points[i]);
						}else if ((label==2)||(label==3)||(label==5)){
							label_rectangular->push_back(cloud_out->points[i]);
						}else{
							label_other->push_back(cloud_out->points[i]);
						}
						vec.push_back(label);
					}
					sort( vec.begin(), vec.end() );
					vec.erase( unique( vec.begin(), vec.end() ), vec.end() );

					for(int i=0;i<vec.size();i++)
				    std::cout<<"label vector: "<<vec[i]<<std::endl;



					pcl::copyPointCloud(*label_circular,*nonzero_ref_circular);
					pcl::copyPointCloud(*label_rectangular,*nonzero_ref_rectangular);
					pcl::copyPointCloud(*label_other,*nonzero_ref_other);
					if(nonzero_ref_circular->size()!=0)
				    pcl::io::savePCDFile("circular.pcd",*nonzero_ref_circular);
					if(nonzero_ref_rectangular->size()!=0)
				    pcl::io::savePCDFile("rectangular.pcd",*nonzero_ref_rectangular);
					if(nonzero_ref_other->size()!=0)
				    pcl::io::savePCDFile("other.pcd",*nonzero_ref_other);

//			        textureless_objects_tracking::cornerFind::Response res_corner;

//					prim_ex.getCornersToPush(bw_image_,res_corner);
//							ros::Duration(2.0);
//					        std::cout<<"res_corner: "<<res_corner.corner.size()<<std::endl;
//					        std::cout<<"res_corner convex: "<<res_corner.corner_convex.size()<<std::endl;

//					if (what == "circular") {

					if(nonzero_ref_circular->size()!=0){

						prim_ex.extractCylinderVector(nonzero_ref_circular,
								result_vector_cylinders, 2);
						prim_ex.extractCircleVector(nonzero_ref_circular,
								result_vector_circles, 1);
						PCL_INFO(
								"number of cylinders: %d \n", result_vector_cylinders.size());
						PCL_INFO(
								"number of circles: %d \n", result_vector_circles.size());

						result_vector = result_vector_cylinders;
						result_vector.insert(result_vector.end(),
								result_vector_circles.begin(),
								result_vector_circles.end());
				}
					if(nonzero_ref_rectangular->size()!=0)
					{
//					} else {
						prim_ex.extractLineVector(nonzero_ref_rectangular,
								result_vector_lines, directions_vector);
						prim_ex.extractCornerVector(nonzero_ref_rectangular,
								result_vector_corners);
						PCL_INFO(
								"number of lines: %d \n", result_vector_lines.size());
						PCL_INFO(
								"number of corners: %d \n", result_vector_corners.size());

//						result_vector = result_vector_lines;
						result_vector.insert(result_vector.end(),
								result_vector_lines.begin(),
								result_vector_lines.end());
						result_vector.insert(result_vector.end(),
								result_vector_corners.begin(),
								result_vector_corners.end());
					}
//					}
				PCL_INFO(
							"number of features: %d \n", result_vector.size());

					tracker_vector_.resize(result_vector.size());

					bool save_feature=false;

					for (int track = 0; track < tracker_vector_.size();
							track++) {

						RefCloudPtr nonzero_ref_final_cloud(new RefCloud);

						pcl::copyPointCloud(*result_vector[track],
								*nonzero_ref_final_cloud);


						if(save_feature){

						for(uint i=0;i<nonzero_ref_final_cloud->points.size();i++){

							PointType searchPointTemp=nonzero_ref_final_cloud->points[i];



							pcl::KdTreeFLANN<PointType> kdtree;

							kdtree.setInputCloud(cloud_to_save_temp);

							std::vector<int> pointIdxRadiusSearch;
							std::vector<float> pointRadiusSquaredDistance;
							float radius=0.002;

							if (kdtree.nearestKSearch(searchPointTemp, 1, pointIdxRadiusSearch,
									pointRadiusSquaredDistance) > 0) {

								cloud_to_save->points[pointIdxRadiusSearch[0]].f=track+1;

								}


							}
						}

//						if (track < directions_vector.size()) {
//							directions_vector[track].normalize();
//
//							step_covariance_[0] *= fabs(
//									directions_vector[track][0]);
//							step_covariance_[1] *= fabs(
//									directions_vector[track][1]);
//							step_covariance_[2] *= fabs(
//									directions_vector[track][2]);
//
//							tracker_vector_[track]->setStepNoiseCovariance(
//									step_covariance_);
//						}

						std::stringstream ss;
						ss << track;
//					 pcl::io::savePCDFileASCII (ss.str()+".pcd", *nonzero_ref_final_cloud);

						PCL_INFO("calculating cog\n");

						RefCloudPtr transed_ref(new RefCloud);
						//pcl::compute3DCentroid<RefPointType> (*nonzero_ref, c);
						pcl::compute3DCentroid<RefPointType>(
								*nonzero_ref_final_cloud, c);

						Eigen::Affine3f trans = Eigen::Affine3f::Identity();
						trans.translation() = Eigen::Vector3f(c[0], c[1], c[2]);
						//pcl::transformPointCloudWithNormals<RefPointType> (*ref_cloud, *transed_ref, trans.inverse());
						//pcl::transformPointCloud<RefPointType> (*nonzero_ref, *transed_ref, trans.inverse());
						pcl::transformPointCloud<RefPointType>(
								*nonzero_ref_final_cloud, *transed_ref,
								trans.inverse());

						CloudPtr transed_ref_downsampled(new Cloud);
						gridSample(transed_ref, *transed_ref_downsampled,
								downsampling_grid_size_);
						tracker_vector_[track]->setReferenceCloud(
								transed_ref_downsampled);
						tracker_vector_[track]->setTrans(trans);
						reference_ = transed_ref;
						tracker_vector_[track]->setMinIndices(
								ref_cloud->points.size() / 2);
					}
					pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr my_points(new pcl::PointCloud<pcl::PointXYZLRegionF>);
				    pcl::io::savePCDFile("features.pcd",*cloud_to_save);
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
		if (cloud_intensity_ != NULL) {

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

		while (!viewer_.wasStopped()){
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}

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
	int feature_lost_;
	pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_; // to store threadpool
	boost::shared_ptr<ParticleFilter> tracker_;
	std::vector<boost::shared_ptr<ParticleFilter> > tracker_vector_;
	std::vector<double> step_covariance_;
	int counter_;
	bool use_convex_hull_;
	bool visualize_non_downsample_;
	bool visualize_particles_;
	double tracking_time_;
	double computation_time_;
	double downsampling_time_;
	double downsampling_grid_size_;
	std::ofstream a_file_;
	cv::Mat bw_image_;

};

