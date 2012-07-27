/*
 * PrimitivesExtract.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: Karol Hausman
 */

#include "textureless_objects_tracking/PrimitivesExtract.h"

inline bool comparison_intens(const pcl::PointXYZI& i,
		const pcl::PointXYZI& j) {
	return (i.intensity > j.intensity);
}

inline bool comparison_curvature(pcl::Normal i, pcl::Normal j) {
	return (i.curvature > j.curvature);
}

template<typename PointType> bool PrimitivesExtract<PointType>::getSegments(
		const CloudConstPtr cloud,
		pcl::PointCloud<pcl::PointXYZLRegion>::Ptr &cloud_out) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(
			new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(*cloud, *cloud_in);
	pcl::io::savePCDFile("cloud_in.pcd",*cloud_in);

	ROS_INFO("Response Clouds Size 1: %d", cloud_in->points.size());
	sensor_msgs::PointCloud2 in_cloud_blob, out_cloud_blob;
	pcl::toROSMsg(*cloud_in, in_cloud_blob);
	ROS_INFO("Response Clouds Size 2: %d", in_cloud_blob.width);
	object_part_decomposition::ClassifyScene srv;
	srv.request.in_cloud = in_cloud_blob;
	srv.request.ID = 0;
	srv.request.params = "-n 5 -s 0";
	if (_segmentation_srv.call(srv)) {
		ROS_INFO("Calling classify scene service");
		out_cloud_blob = srv.response.out_cloud;
		pcl::fromROSMsg(out_cloud_blob, *cloud_out);
		ROS_INFO("Response Clouds Size: %d", cloud_out->points.size());
		pcl::io::savePCDFile("service_call_result.pcd", *cloud_out);
		return true;
	} else {
		ROS_ERROR("Failed to call service classify_scene");
		return false;
	}
}

template<typename PointType> bool PrimitivesExtract<PointType>::getCornersToPush(
		cv::Mat& topview,
		textureless_objects_tracking::cornerFind::Response& res) {
	textureless_objects_tracking::cornerFind::Request req;
	IplImage temp(topview);
//	cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
//	cv::imshow("Display window", topview);
//
//	cv::waitKey(0);
	sensor_msgs::ImagePtr imgptr = sensor_msgs::CvBridge::cvToImgMsg(&temp);
	req.image = *imgptr;
	_corner_finder.call(req, res);

	return true;
}

template<typename PointType> void PrimitivesExtract<PointType>::computeNormals(
		const CloudConstPtr cloud,
		pcl::PointCloud<pcl::Normal>::Ptr normals_cloud) {
	pcl::NormalEstimation<PointType, pcl::Normal> norm_est;
	norm_est.setSearchMethod(KdTreePtr(new KdTree));
	norm_est.setInputCloud(cloud);
	norm_est.setRadiusSearch(find_normals_radius_search_);
	norm_est.compute(*normals_cloud);
}

template<typename PointType> void PrimitivesExtract<PointType>::findBoundaries(
		const CloudConstPtr cloud, Cloud &result) {

	pcl::PointCloud<pcl::Normal>::Ptr temp_normals(
			new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_cloud(
			new pcl::PointCloud<pcl::Normal>);

	computeNormals(cloud, normals_cloud);

	pcl::copyPointCloud(*normals_cloud, *temp_normals);

	std::sort(temp_normals->points.begin(), temp_normals->points.end(),
			comparison_curvature);
	int place = (int) (temp_normals->points.size() * best_curv_percent_);
	pcl::Normal tresh_point = *(temp_normals->points.begin() + place);
	float treshold = tresh_point.curvature;
	PCL_DEBUG("Curvature treshold for corners: %f \n", treshold);

	for (size_t i = 0; i < normals_cloud->size(); ++i) {
		if (normals_cloud->points[i].curvature > treshold)
			result.push_back(cloud->points.at(i));
	}

	result.width = result.points.size();
	result.height = 1;
	result.is_dense = false;

}

template<typename PointType> bool PrimitivesExtract<PointType>::extractLineVector(
		const CloudConstPtr& input, std::vector<CloudPtr>& result,
		std::vector<Eigen::Vector3f> &directions_vector, int lines_number) {

	CloudPtr cloud_boundaries(new Cloud);

	int size;

	std::vector<CloudPtr> vector_lines;
	std::vector<pcl::ModelCoefficients::Ptr> coefficients;

	findBoundaries(input, *cloud_boundaries);
	extractLines(cloud_boundaries, vector_lines, coefficients, lines_number);

	for (int number = 0; number < vector_lines.size(); number++) {
		CloudPtr thin_line(new Cloud);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		thin_line = vector_lines[number];
		CloudPtr thick_line(new Cloud);

		for (size_t j = 0; j < thin_line->points.size(); j++) {

			extractNeighbor(input, thin_line->points[j], inliers, size);
		}
		std::sort(inliers->indices.begin(), inliers->indices.end());
		inliers->indices.erase(
				std::unique(inliers->indices.begin(), inliers->indices.end()),
				inliers->indices.end());

		for (size_t i = 0; i < inliers->indices.size(); i++) {
			PointType point = input->points[inliers->indices[i]];
			thick_line->points.push_back(point);
		}

		removePointsAroundLine(thick_line, *thick_line, *thin_line,
				coefficients[number]);
		int planes_number = countPlanes(thick_line);
		if ((planes_number == 2) || (planes_number == 3)
				|| (planes_number == 4)) {
			Eigen::Vector3f line_direction;
			lineDirectionPCA(thin_line, line_direction);
			result.push_back(thick_line);
			directions_vector.push_back(line_direction);
		} else
			PCL_INFO(
					"Line deleted because of too many planes. Planes number:  %d \n", planes_number);

	}
	if (result.size() == 0)
		return false;
	else
		return true;

}

template<typename PointType> bool PrimitivesExtract<PointType>::extractCornerVector(
		const CloudConstPtr cloud_input, std::vector<CloudPtr>& result,
		int number) {

	CloudPtr corners(new Cloud);
	CloudPtr corners_debug(new Cloud);
	int size;

	extractCorners(cloud_input, *corners, *corners_debug, number);

	for (size_t j = 0; j < corners->points.size(); j++) {
		CloudPtr augmented_corner(new Cloud);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		extractNeighbor(cloud_input, corners->points[j], inliers, size);

		augmented_corner->points.push_back(corners->points[j]);

		for (size_t i = 0; i < inliers->indices.size(); i++) {
			PointType point = cloud_input->points[inliers->indices[i]];
			augmented_corner->points.push_back(point);

		}
		int planes_number = countPlanes(augmented_corner);
//		if ((planes_number == 4) || (planes_number == 3))
		result.push_back(augmented_corner);
//		else
//			PCL_INFO(
//					"Corner deleted because of too many planes. Planes number:  %d \n", planes_number);

	}
	if (result.size() == 0)
		return false;
	else
		return true;

}

template<typename PointType> void PrimitivesExtract<PointType>::removePrimitive(
		const CloudConstPtr &cloud, pcl::PointIndices::Ptr &indices_to_remove,
		Cloud &result) {

	pcl::ExtractIndices<PointType> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(indices_to_remove);
	extract.setNegative(true);

	extract.filter(result);

}

template<typename PointType> bool PrimitivesExtract<PointType>::extractLines(
		const CloudConstPtr &cloud, std::vector<CloudPtr> &result_vector,
		std::vector<pcl::ModelCoefficients::Ptr> &coefficients_vector,
		int lines_number) {

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::PointIndices::Ptr extended_inliers(new pcl::PointIndices);

	CloudPtr nonline_cloud(new Cloud);

	if (cloud->size() == 0)
		return false;
	pcl::copyPointCloud(*cloud, *nonline_cloud);

	for (int number = 0;; number++) {
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

		if (lines_number > 0)
			if (number >= lines_number)
				break;
		pcl::SACSegmentation<PointType> seg;
		seg.setOptimizeCoefficients(true);

		seg.setModelType(pcl::SACMODEL_LINE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(line_distance_tresh_);

		seg.setInputCloud(nonline_cloud);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() < min_line_inliers_) {
			PCL_DEBUG(
					"Could not estimate a line model for the given dataset.");
			PCL_INFO(
					"Number of line inliers: %d  is smaller than treshold: %f \n", (int)inliers->indices.size(), min_line_inliers_);
			break;
		}
		if (coefficients != NULL)
			coefficients_vector.push_back(coefficients);
		CloudPtr one_line(new Cloud);
		for (size_t i = 0; i < inliers->indices.size(); i++) {
			PointType point = nonline_cloud->points[inliers->indices[i]];

			one_line->points.push_back(point);

			float radius = eliminate_line_neigh_radius_;
			int size;
			extractNeighbor(nonline_cloud, point, extended_inliers, size,
					radius);
			extended_inliers->indices.push_back(i);

		}
		std::sort(extended_inliers->indices.begin(),
				extended_inliers->indices.end());
		extended_inliers->indices.erase(
				std::unique(extended_inliers->indices.begin(),
						extended_inliers->indices.end()),
				extended_inliers->indices.end());
		result_vector.push_back(one_line);
		removePrimitive(nonline_cloud, extended_inliers, *nonline_cloud);
	}
	if (result_vector.size() == 0)
		return false;
	else
		return true;

}

template<typename PointType> void PrimitivesExtract<PointType>::extractNeighbor(
		const CloudConstPtr cloud, PointType &searchPoint,
		pcl::PointIndices::Ptr &inliers, int& size, float radius) {

	pcl::KdTreeFLANN<PointType> kdtree;

	kdtree.setInputCloud(cloud);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	if (radius == 0)
		radius = extract_neigh_radius_;
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
			pointRadiusSquaredDistance) > 0) {
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)

			inliers->indices.push_back(pointIdxRadiusSearch[i]);

	}

	size = (int) pointIdxRadiusSearch.size();

}

template<typename PointType> bool PrimitivesExtract<PointType>::extractPlane(
		const CloudConstPtr cloud, pcl::PointIndices::Ptr &inliers) {

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	if (cloud->size() == 0)
		return false;

	pcl::SACSegmentation<PointType> seg;
	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(plane_distance_treshold_);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() < min_plane_inliers_) {
		PCL_DEBUG( "Could not estimate a plane model for the given dataset.");
		return false;
	}

	return true;

}

template<typename PointType> int PrimitivesExtract<PointType>::countPlanes(
		const CloudConstPtr cloud) {
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	int number = 0;

	CloudPtr nonplane_cloud(new Cloud);
	pcl::copyPointCloud(*cloud, *nonplane_cloud);
	for (u_int i = 0; i < 6; i++) {

		inliers.reset(new pcl::PointIndices);

		if (extractPlane(nonplane_cloud, inliers)) {

			removePrimitive(nonplane_cloud, inliers, *nonplane_cloud);
			number++;
		} else
			return number;
	}

	return number;

}
template<typename PointType> bool PrimitivesExtract<PointType>::getTopView(
		const CloudConstPtr cloud, cv::Mat& topview) {

	CloudPtr cloud_in_virt_cam(new Cloud);
	tf::Transform full_tf = virt_cam_rot_ * virt_cam_transl_;

	Eigen::Affine3d transform_eigen;
	tf::TransformTFToEigen(full_tf, transform_eigen);
	Eigen::Matrix4d transform_eigen3(transform_eigen.matrix());
	Eigen::Matrix4f transform_eigen3f = transform_eigen3.cast<float>();
	pcl::transformPointCloud(*cloud, *cloud_in_virt_cam, transform_eigen3f);

	cv::Mat mask = cv::Mat::zeros(cv::Size(_cam_info->width, _cam_info->height),
			CV_8U);

	std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA> >::iterator iter;
	for (iter = cloud_in_virt_cam->points.begin();
			iter != cloud_in_virt_cam->points.end(); ++iter) {
		pcl::PointXYZRGBA& point = *iter;

		if (isnan(point.x) || isnan(point.y) || isnan(point.z))
			continue;
		cv::Point3d p3d(point.x, point.y, point.z);
		cv::Point2d p2d;
		model_.project3dToPixel(p3d, p2d);
		int x = round(p2d.x);
		int y = round(p2d.y);
		if ((x > mask.cols - 1) || (x < 0) || (y > mask.rows - 1) || (y < 0))
			continue;
		mask.at<unsigned char>(y, x) = 255;
	}

	cv::morphologyEx(mask, mask, CV_MOP_CLOSE,
			getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)),
			cv::Point(-1, -1), 3);

	topview = mask;
	return true;

}

template<typename PointType> bool PrimitivesExtract<PointType>::get3dPoints(
		const textureless_objects_tracking::cornerFind::Response& res) {
	Eigen::Vector3f table_normal(plane_coefficients_->values[0],
			plane_coefficients_->values[1], plane_coefficients_->values[2]);

	convex_corners_.reserve(res.corner_convex.size());
	convex_corners_.width = res.corner_convex.size();
	convex_corners_.height = 1;
	for (int i = 0; i < res.corner_convex.size(); ++i) {
		cv::Point2d p2d(res.corner_convex[i].x, res.corner_convex[i].y);
		cv::Point3d p3d;

//		get the ray of the pinhole camera
		model_.projectPixelTo3dRay(p2d, p3d);
		Eigen::Vector3f ray(p3d.x, p3d.y, p3d.z);

//		distance to the corner to push from the virtual camera
		float t = -1.0 * plane_coefficients_->values[3]
				/ (table_normal.dot(ray));

//		vector to the corner
		Eigen::Vector3f intersec = t * ray;

		PointType p;
		p.getArray3fMap() = intersec;
		convex_corners_.push_back(p);
	}

	Eigen::Affine3d transform_eigen;

	tf::Transform full_tf = virt_cam_rot_.inverse()
			* virt_cam_transl_.inverse();

	tf::TransformTFToEigen(full_tf, transform_eigen);
	Eigen::Matrix4d transform_eigen3(transform_eigen.matrix());
	Eigen::Matrix4f transform_eigen3f = transform_eigen3.cast<float>();

//	CONCAVE CORNERS

	concave_corners_.reserve(res.corner.size());

	concave_corners_.width = res.corner.size();
	concave_corners_.height = 1;
	for (int i = 0; i < res.corner.size(); ++i) {
		cv::Point2d p2d(res.corner[i].x, res.corner[i].y);
		cv::Point3d p3d;

//		get the ray of the pinhole camera
		model_.projectPixelTo3dRay(p2d, p3d);
		Eigen::Vector3f ray(p3d.x, p3d.y, p3d.z);

//		distance to the corner to push from the virtual camera
		float t = -1.0 * plane_coefficients_->values[3]
				/ (table_normal.dot(ray));

//		vector to the corner
		Eigen::Vector3f intersec = t * ray;

		PointType p;
		p.getArray3fMap() = intersec;
		concave_corners_.push_back(p);
	}



//	pcl::transformPointCloud(  grasp_points, grasp_points, transform_eigen3f );

	return true;
}

template<typename PointType> void PrimitivesExtract<PointType>::getAll3DCornersFromService(const CloudConstPtr cloud,Cloud &concave_result,Cloud &convex_result){

	cv::Mat top_image(640, 480, CV_8U);
	getTopView(cloud, top_image);
	textureless_objects_tracking::cornerFind::Response res_corner;
	getCornersToPush(top_image, res_corner);
	get3dPoints(res_corner);
	pcl::copyPointCloud(concave_corners_,concave_result);
	pcl::copyPointCloud(convex_corners_,convex_result);

}


template<typename PointType> bool PrimitivesExtract<PointType>::extractCorners(
		const CloudConstPtr cloud, Cloud &result, Cloud &result_debug,
		int number) {

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity(
			new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity_after_treshhold(
			new pcl::PointCloud<pcl::PointXYZI>);

	pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI>* harris3D =
			new pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI>(
					pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI>::HARRIS);
	harris3D->setNonMaxSupression(true);
//		 harris3D->setThreshold(0.0009);
//		 harris3D->setThreshold(0.00011);

	harris3D->setRadius(radius_harris_);
	harris3D->setRadiusSearch(radius_search_harris_);
	harris3D->setInputCloud(cloud);
	harris3D->setRefine(false);
	harris3D->setMethod(
			pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI>::HARRIS);
	harris3D->compute(*cloud_intensity);

	if (cloud_intensity->size() > 0) {
		std::sort(cloud_intensity->points.begin(),
				cloud_intensity->points.end(), comparison_intens);
		if (number == 0) {
			int place = (int) (cloud_intensity->points.size()
					* best_intens_percent_);
			pcl::PointXYZI tresh_point = *(cloud_intensity->points.begin()
					+ place);
			float treshold = tresh_point.intensity;

			for (size_t i = 0; i < cloud_intensity->size(); ++i) {
				if (cloud_intensity->points[i].intensity > treshold)
					cloud_intensity_after_treshhold->points.push_back(
							cloud_intensity->points.at(i));
			}
		} else {
			if (number > cloud_intensity->size()) {
				PCL_DEBUG("Number of corners is bigger than found corners.");
				return false;
			} else {

				for (size_t i = 0; i < number; ++i) {
					cloud_intensity_after_treshhold->points.push_back(
							cloud_intensity->points[i]);
				}
			}
		}

		pcl::copyPointCloud(*cloud_intensity_after_treshhold, result);
		pcl::copyPointCloud(result, result_debug);

		if (convex_corners_only_) {

			cv::Mat top_image(640, 480, CV_8U);
			getTopView(cloud, top_image);
			textureless_objects_tracking::cornerFind::Response res_corner;
			getCornersToPush(top_image, res_corner);
			get3dPoints(res_corner);

			int K = 1;

			CloudPtr result_projected(new Cloud);

			pcl::ProjectInliers<PointType> proj;
			proj.setModelType(pcl::SACMODEL_PLANE);
			proj.setInputCloud(result.makeShared());
			proj.setModelCoefficients(plane_coefficients_);
			proj.filter(*result_projected);

			std::vector<int> pointIdxNKNSearch;
			std::vector<float> pointNKNSquaredDistance;

			KdTreePtr tree(new KdTree());
			tree->setInputCloud(result_projected);
			Cloud result_convex_only;

			for (uint i = 0; i < convex_corners_.points.size(); i++) {

				PointType searchPoint = convex_corners_.points[i];

				pointIdxNKNSearch.resize(1);
				pointNKNSquaredDistance.reserve(1);

				if (tree->nearestKSearch(searchPoint, K, pointIdxNKNSearch,
						pointNKNSquaredDistance) > 0) {
					for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j) {
						if ((pointIdxNKNSearch[j] >= 0)
								&& (pointNKNSquaredDistance[j] < max_distance_from_corner_service_)
								&& (pointIdxNKNSearch[j] < result.points.size())) {
							result_convex_only.push_back(
									result.points[pointIdxNKNSearch[j]]);
						}
					}

				}



			}
			std::cout << "Number of result_convex_only"
					<< result_convex_only.size() << std::endl;
			result.clear();
			pcl::copyPointCloud(result_convex_only, result);

			/*
			 std::vector<int> neighbor_num;

			 for (size_t j = 0; j < result.points.size(); j++) {

			 pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			 int size;
			 extractNeighbor(cloud, result.points[j], inliers, size);

			 if (size > 0)
			 neighbor_num.push_back(size);

			 }
			 double avg = 0;
			 std::vector<int>::iterator it;
			 for (it = neighbor_num.begin(); it != neighbor_num.end(); it++)
			 avg += *it;
			 avg /= neighbor_num.size();

			 if (neighbor_num.size() == result.points.size()) {
			 for (int n = 0; n < neighbor_num.size(); n++) {
			 if (neighbor_num[n] > avg) {
			 neighbor_num.erase(neighbor_num.begin() + n);
			 result.points.erase(result.points.begin() + n);
			 }
			 }
			 } else
			 return false;

			 */} else {
			return false;
		}
	}
	result.width = result.points.size();
	result.height = 1;
	result.is_dense = false;

	return true;

}

template<typename PointType> void PrimitivesExtract<PointType>::euclidianClustering(
		CloudPtr& cloudForEuclidianDistance,
		std::vector<pcl::PointIndices>& cluster_indices,
		float euclidian_cluster_tolerance) {
	KdTreePtr tree(new KdTree());
	tree->setInputCloud(cloudForEuclidianDistance);

//	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointType> ec;
	ec.setClusterTolerance(euclidian_cluster_tolerance);
	ec.setMinClusterSize(euclidian_min_cluster_size_);
	ec.setMaxClusterSize(euclidian_max_cluster_size_);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloudForEuclidianDistance);
	ec.extract(cluster_indices);
}

template<typename PointType> void PrimitivesExtract<PointType>::removePointsAroundLine(
		const CloudConstPtr &cloud, Cloud &result, Cloud &line,
		pcl::ModelCoefficients::Ptr &coefficients) {

	CloudPtr cloud_projected(new Cloud);

	pcl::ProjectInliers<PointType> proj;
	proj.setModelType(pcl::SACMODEL_LINE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);

	Eigen::Vector4f c;
	pcl::compute3DCentroid<RefPointType>(line, c);
	float dist, dist2;
	float max_dist = 0;
	float max_dist2 = 0;
	float max_dist_general = 0;

	PointType borderPoint1 = line.points.at(1);
	PointType borderPoint2 = line.points.at(1);
	PointType cloudPoint;
	PointType linePoint;

	Cloud temp_cloud;

	pcl::copyPointCloud(*cloud, temp_cloud);

	for (size_t i = 0; i < line.size(); ++i) {
		linePoint = line.points.at(i);
		dist = sqrt(
				(linePoint.x - c[0]) * (linePoint.x - c[0])
						+ (linePoint.y - c[1]) * (linePoint.y - c[1])
						+ (linePoint.z - c[2]) * (linePoint.z - c[2]));
		if (dist > max_dist) {
			max_dist = dist;
			borderPoint1 = linePoint;
		}

	}

	for (size_t i = 0; i < line.size(); ++i) {
		linePoint = line.points.at(i);
		dist2 = sqrt(
				(linePoint.x - borderPoint1.x) * (linePoint.x - borderPoint1.x)
						+ (linePoint.y - borderPoint1.y)
								* (linePoint.y - borderPoint1.y)
						+ (linePoint.z - borderPoint1.z)
								* (linePoint.z - borderPoint1.z));
		if (dist2 > max_dist2) {
			max_dist2 = dist2;
			borderPoint2 = linePoint;
		}

	}

	max_dist_general = shrink_line_percent_
			* sqrt(
					(borderPoint1.x - borderPoint2.x)
							* (borderPoint1.x - borderPoint2.x)
							+ (borderPoint1.y - borderPoint2.y)
									* (borderPoint1.y - borderPoint2.y)
							+ (borderPoint1.z - borderPoint2.z)
									* (borderPoint1.z - borderPoint2.z));

	for (size_t i = 0; i < cloud_projected->size(); i++) {
		cloudPoint = cloud_projected->points.at(i);
		float distance = sqrt(
				(cloudPoint.x - borderPoint1.x)
						* (cloudPoint.x - borderPoint1.x)
						+ (cloudPoint.y - borderPoint1.y)
								* (cloudPoint.y - borderPoint1.y)
						+ (cloudPoint.z - borderPoint1.z)
								* (cloudPoint.z - borderPoint1.z));
		float distance2 = sqrt(
				(cloudPoint.x - borderPoint2.x)
						* (cloudPoint.x - borderPoint2.x)
						+ (cloudPoint.y - borderPoint2.y)
								* (cloudPoint.y - borderPoint2.y)
						+ (cloudPoint.z - borderPoint2.z)
								* (cloudPoint.z - borderPoint2.z));

		if ((distance > max_dist_general) || (distance2 > max_dist_general)) {
			temp_cloud.points.at(i).x = 0;
			temp_cloud.points.at(i).y = 0;
			temp_cloud.points.at(i).z = 0;
		}

	}

	for (int i = 0; i < temp_cloud.size(); i++) {
		if ((temp_cloud.points[i].x == 0) && (temp_cloud.points[i].y == 0)
				&& (temp_cloud.points[i].z == 0)) {
			temp_cloud.erase(temp_cloud.begin() + i);
			i--;
		}
	}

	if (!euclidian_clustering_after_line_projection_) {
		result.clear();
		pcl::copyPointCloud(temp_cloud, result);

	} else {
		CloudPtr cloudForEuclidianDistance(new Cloud);
		pcl::copyPointCloud(temp_cloud, *cloudForEuclidianDistance);

		std::vector<pcl::PointIndices> cluster_indices;

		euclidianClustering(cloudForEuclidianDistance, cluster_indices,
				euclidian_line_cluster_tolerance_);

		result.clear();
		for (size_t i = 0; i < cluster_indices[0].indices.size(); i++) {
			PointType point =
					cloudForEuclidianDistance->points[cluster_indices[0].indices[i]];
			result.points.push_back(point);
		}

	}
	result.width = result.points.size();
	result.height = 1;
	result.is_dense = false;

}

template<typename PointType> bool PrimitivesExtract<PointType>::extractCylinderVector(
		const CloudConstPtr &cloud, std::vector<CloudPtr> &result,
		int cylinders_number) {
	return extractCircular(cloud, result, cylinders_number, "cylinder");
}

template<typename PointType> bool PrimitivesExtract<PointType>::extractCircleVector(
		const CloudConstPtr &cloud, std::vector<CloudPtr> &result,
		int cylinders_number) {
	return extractCircular(cloud, result, cylinders_number, "circle");
}

template<typename PointType> bool PrimitivesExtract<PointType>::extractCircular(
		const CloudConstPtr &cloud, std::vector<CloudPtr> &result,
		int cylinders_number, std::string what) {

	CloudPtr noncylinder_cloud(new Cloud);

	pcl::copyPointCloud(*cloud, *noncylinder_cloud);

	for (int number = 0;; number++) {

		if (cylinders_number > 0) {
			if (number >= cylinders_number)
				break;
		}

		CloudPtr one_cylinder_cloud(new Cloud);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		if (cloud->size() == 0)
			return false;

		pcl::PointCloud<pcl::Normal>::Ptr normals_cloud(
				new pcl::PointCloud<pcl::Normal>);

		computeNormals(noncylinder_cloud, normals_cloud);

		pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;
		seg.setOptimizeCoefficients(true);

		if (what == "cylinder")
			seg.setModelType(pcl::SACMODEL_CYLINDER);
		else
			seg.setModelType(pcl::SACMODEL_CIRCLE3D);

		seg.setMethodType(pcl::SAC_RANSAC);
		if (what == "cylinder")
			seg.setDistanceThreshold(cylinder_distance_treshold_);
		else
			seg.setDistanceThreshold(circle_distance_treshold_);

		if (what == "cylinder")
			seg.setRadiusLimits(cylinder_min_radius_, cylinder_max_radius_);
		else
			seg.setRadiusLimits(circle_min_radius_, circle_max_radius_);

		seg.setInputNormals(normals_cloud);
		seg.setInputCloud(noncylinder_cloud);
		seg.setMaxIterations(10000);

		seg.segment(*inliers, *coefficients);

		if (what == "cylinder") {
			if (inliers->indices.size() < cylinder_min_inliers_) {
				PCL_DEBUG(
						"Could not estimate a cylinder model for the given dataset.");
				PCL_INFO(
						"Number of cylinder inliers: %d  is smaller than treshold: %f \n", (int)inliers->indices.size(), cylinder_min_inliers_);

				break;
			}
		} else {
			if (inliers->indices.size() < circle_min_inliers_) {
				PCL_DEBUG(
						"Could not estimate a 3D circle model for the given dataset.");
				PCL_INFO(
						"Number of circle inliers: %d  is smaller than treshold: %f \n", (int)inliers->indices.size(), circle_min_inliers_);

				break;
			}
		}

		for (size_t i = 0; i < inliers->indices.size(); i++) {
			PointType point = noncylinder_cloud->points[inliers->indices[i]];
			one_cylinder_cloud->points.push_back(point);
		}

		if (euclidian_clustering_after_circular_extraction_) {

			std::vector<pcl::PointIndices> cluster_indices;

			if (what == "cylinder")
				euclidianClustering(one_cylinder_cloud, cluster_indices,
						euclidian_cylinder_cluster_tolerance_);
			else
				euclidianClustering(one_cylinder_cloud, cluster_indices,
						euclidian_circle_cluster_tolerance_);

			CloudPtr one_cylinder_after_euclidian_cloud(new Cloud);

			for (size_t i = 0; i < cluster_indices[0].indices.size(); i++) {
				PointType point =
						one_cylinder_cloud->points[cluster_indices[0].indices[i]];
				one_cylinder_after_euclidian_cloud->points.push_back(point);
			}

			result.push_back(one_cylinder_after_euclidian_cloud);
			pcl::PointIndices::Ptr cluster_inliers(new pcl::PointIndices);
			for (size_t i = 0; i < cluster_indices[0].indices.size(); i++) {
				int index = inliers->indices[cluster_indices[0].indices[i]];
				cluster_inliers->indices.push_back(index);
			}

			removePrimitive(noncylinder_cloud, cluster_inliers,
					*noncylinder_cloud);
		} else {
			result.push_back(one_cylinder_cloud);
			removePrimitive(noncylinder_cloud, inliers, *noncylinder_cloud);
		}

	}

	if (result.size() == 0)
		return false;
	else
		return true;

}

template<typename PointType> void PrimitivesExtract<PointType>::lineDirectionPCA(
		const CloudConstPtr &cloud, Eigen::Vector3f &direction) {

	Eigen::Vector4f centroid;

	EIGEN_ALIGN16
	Eigen::Vector3f eigen_values;
	EIGEN_ALIGN16
	Eigen::Matrix3f eigen_vectors;
	Eigen::Matrix3f cov;
	Eigen::Vector3f eigen_vector1;
	Eigen::Vector3f eigen_vector2;
	Eigen::Vector3f vector3rd;

	pcl::compute3DCentroid(*cloud, centroid);

	pcl::computeCovarianceMatrixNormalized(*cloud, centroid, cov);
	pcl::eigen33(cov, eigen_vectors, eigen_values);

	eigen_vector1(0) = eigen_vectors(0, 2);
	eigen_vector1(1) = eigen_vectors(1, 2);
	eigen_vector1(2) = eigen_vectors(2, 2);
	eigen_vector2(0) = eigen_vectors(0, 1);
	eigen_vector2(1) = eigen_vectors(1, 1);
	eigen_vector2(2) = eigen_vectors(2, 1);

	direction = eigen_vector1;

}

