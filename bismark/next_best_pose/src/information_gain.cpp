#include <math.h>
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/features/normal_3d.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/search/kdtree.h"
#include "pcl/search/flann_search.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/common/common.h"
#include "pcl/common/angles.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "boost/thread/thread.hpp"
#include "pcl/common/transforms.h"
#include "pcl/common/geometry.h"
#include "pcl/sample_consensus/sac_model_plane.h"




using namespace std;

typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointXYZ PointT;


int npoints_sphere = 5000;

void Normalise(pcl::PointXYZ *p,double r)
{
   double l;

   l = r / sqrt(p->x*p->x + p->y*p->y + p->z*p->z);
   p->x *= l;
   p->y *= l;
   p->z *= l;
}

pcl::PointCloud<pcl::PointXYZ> generate_sphere (float r){

   pcl::PointXYZ p[npoints_sphere],p1,p2;

   // Create the initial random cloud
   for (int i=0;i<npoints_sphere;i++) {
	   p[i].x = (rand()%1000)-500;
	   p[i].y = (rand()%1000)-500;
	   p[i].z = (rand()%1000)-500;
	   Normalise(&p[i],r);

	}


   pcl::PointCloud<pcl::PointXYZ> cloud;
   pcl::PointXYZ point;
   cloud.width = npoints_sphere;
   cloud.height = 1;

   for(int i=0; i<npoints_sphere; i++)
   {
	   point.x=p[i].x;
	   point.y=p[i].y;
	   point.z=p[i].z;
	   cloud.points.push_back(point);

   }
   return(cloud);
}




pcl::PointCloud<pcl::PointXYZ>::ConstPtr create_workspace (Eigen::Vector4f object_centroid, pcl::PointCloud<PointT>::Ptr extracted_objects,
																pcl::ModelCoefficients table_coeff){

    pcl::PointCloud<pcl::PointXYZ>::Ptr robot_sphere (new pcl::PointCloud<pcl::PointXYZ>),
										object_sphere (new pcl::PointCloud<pcl::PointXYZ>),
										cloud (new pcl::PointCloud<pcl::PointXYZ>),
										workspace_cloud (new pcl::PointCloud<pcl::PointXYZ>),
										trans_object_cloud (new pcl::PointCloud<pcl::PointXYZ>),
										trans_robot_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //paramter for both spheres (object sphere = nearest distance to objects; workspace sphere = approx. Workspace of robot)
    float min_dist_objects = 0.6;
    float radius_workspace = 0.6;
    Eigen::Vector4f workspace_centroid;
    workspace_centroid << 0.5,-0.3,1.2,0;

    //create both spheres
    *object_sphere = generate_sphere(min_dist_objects);
    *robot_sphere = generate_sphere(radius_workspace);

    //Transform both spheres to their centroids
	Eigen::Vector3f object_trans;
	object_trans = object_centroid.head(3);
	transformPointCloud(*object_sphere, *trans_object_cloud,object_trans,Eigen::Quaternionf(0,0,0,1));

	Eigen::Vector3f ws_trans;
	ws_trans = workspace_centroid.head(3);
	transformPointCloud(*robot_sphere, *trans_robot_cloud,ws_trans,Eigen::Quaternionf(0,0,0,1));

	//put both spheres togehter
	*cloud = *trans_object_cloud + *trans_robot_cloud;



	//compute nearest point (from object cloud) to robot
	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(*extracted_objects, min_pt,max_pt);


	//compare distance to centroid and erase points which are not in workspace
	for(int i=0; i<npoints_sphere*2; i++){
		pcl::PointXYZ p , ws_center_point, object_center_point, min_plane;
		p = cloud->points[i];
		ws_center_point.x = ws_trans.coeff(0);
		ws_center_point.y = ws_trans.coeff(1);
		ws_center_point.z = ws_trans.coeff(2);
		object_center_point.x = object_trans.coeff(0);
		object_center_point.y = object_trans.coeff(1);
		object_center_point.z = object_trans.coeff(2);


		//computing distance from each point to the centroids
		float dist_to_workspace_center = pcl::euclideanDistance(p,ws_center_point);
		float dist_to_object_center = pcl::euclideanDistance(p,object_center_point);

		//project point to min distance plane if its nearer to the robot
		min_plane.x = min_pt.coeff(0);
		if(p.x<min_plane.x)
			p.x = min_plane.x;

		//project the point onto the table its under it
		float dist_to_table;
		dist_to_table = (-table_coeff.values[3]-(table_coeff.values[0]*p.x)-(table_coeff.values[1]*p.y))/table_coeff.values[2];
		if(p.z<dist_to_table)
			p.z=dist_to_table;

		//buliding pointcloud regarding to the max distance to centroid
		if(dist_to_workspace_center <= radius_workspace)
			workspace_cloud->points.push_back(p);

		if(dist_to_object_center < min_dist_objects)
			workspace_cloud->points.pop_back();

	}

	pcl::PCDWriter writer_icp;
	writer_icp.write ("workspace.pcd", *workspace_cloud, true);


	return(workspace_cloud);
}






boost::shared_ptr<pcl::visualization::PCLVisualizer> Vis (pcl::PointCloud<PointT>::ConstPtr cloud, Eigen::Vector4f centroid_objects,
															Eigen::Vector4f centroid_table, pcl::ModelCoefficients table_coeff, pcl::PointCloud<pcl::PointXYZ>::ConstPtr workspace_cloud)
{
	//Setup the visualization (input cloud and rendering parameters)
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
	viewer->addPointCloud<PointT> (cloud,rgb,"sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem (0.3);

	//add the next pose point cloud
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (workspace_cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(workspace_cloud,single_color,"next_pose");


//	//Setup of sphere coefficients
//    pcl::ModelCoefficients sphere_mindist_coeff;
//    sphere_mindist_coeff.values.resize (4);	// create size (4 value)
//    sphere_mindist_coeff.values[0] = centroid_objects.coeff(0); // x transformation
//    sphere_mindist_coeff.values[1] = centroid_objects.coeff(1); // y transformation
//    sphere_mindist_coeff.values[2] = centroid_objects.coeff(2);	// z transformation
//    sphere_mindist_coeff.values[3] = 0.6; // radius
//
//    pcl::ModelCoefficients sphere_robot_coeff;
//    sphere_robot_coeff.values.resize (4);	// create size (4 value)
//    sphere_robot_coeff.values[0] = 0.5; // x transformation
//    sphere_robot_coeff.values[1] = -0.3; // y transformation
//    sphere_robot_coeff.values[2] = 1.2;	// z transformation
//    sphere_robot_coeff.values[3] = 0.6; // radius



    //compute orientation of the table
    Eigen::Vector3f n, v, u;
    n << table_coeff.values[0],table_coeff.values[1],table_coeff.values[2];
    v = n.unitOrthogonal();
    u = n.cross(v);
    Eigen::Matrix3f m(3,3);
    m << v,u,n;
    //translation of table
    Eigen::Vector3f table_pos;
    table_pos << centroid_table.coeff(0),centroid_table.coeff(1),centroid_table.coeff(2);
    //table size
    double table_height = 3.0;
    double table_width = 3.0;
    double table_depth = 0.002;

	//Parameter for plane at nearest point in x-direction
    pcl::ModelCoefficients min_plane;
    //nearest point to base (min_pt)
	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(*cloud, min_pt,max_pt);
    min_plane.values.resize (10); // create size (10 value)
    min_plane.values[0] = min_pt.coeff(0); // x transformation
    min_plane.values[1] = min_pt.coeff(1); // y transformation
    min_plane.values[2] = min_pt.coeff(2); // z transformation
    min_plane.values[3] = 0.0;	// x Quaternion
    min_plane.values[4] = 0.0;	// y Quaternion
    min_plane.values[5] = 0.0; // z Quaternion
    min_plane.values[6] = 0.1;	// w Quaternion
    min_plane.values[7] = 0.002;
    min_plane.values[8] = 2.0;
    min_plane.values[9] = 1.0;

	//add shapes to the cloud
//    viewer->addPlane(table_coeff,"table");
//    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,1,"table");
    //add plane which indicates the nearest point to robot
    viewer->addCube(min_plane,"min_plane");
    //add the sphere which is indicating the min distance to the objects
    //viewer->addSphere(sphere_mindist_coeff,"sphere_mindist");
    //add the sphere which is indicating workspace of the robot
    //viewer->addSphere(sphere_robot_coeff,"sphere_robot");
    //add the table
	viewer->addCube(table_pos,Eigen::Quaternionf(m),table_width,table_height,table_depth, "table");


	//parameter for visualization (color, line thickness)
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"min_plane");
//	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"sphere_mindist");
//	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,255,0,"sphere_robot");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,10,"table");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,2,"table");


	//Initialize camera parameters with some default values.
	viewer->initCameraParameters ();
	return (viewer);
}





int main(int argc, char** argv){


	//filter paramters
	float filter_max = 1.5;
	float filter_min = 0;
	float filter_y_max = 1.3;
	float filter_y_min = -1;

	//normal estimation parameter
	float k_search = 50;

	//segmentation parameter (table)
	float seg_distance_threshold = 0.01;
	float seg_normal_distance_weight = 0.05;
	float seg_iter = 1000;
	float seg_probability = 0.99;
	float eps_angle = 10;

	//prism parameter
	float cluster_min_height = -0.2;
	float cluster_max_height = -0.02;


	//all needed objects
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::ConvexHull<PointT> convhull_;
	pcl::NormalEstimation<PointT, pcl::Normal> ne_;												//Normal estimation
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_;									//Planar segmentation object
	pcl::ProjectInliers<PointT> proj_;															//Inlier projection object
	pcl::ExtractIndices<PointT> extract_;														//Extract object (table)
	pcl::ExtractPolygonalPrismData<PointT> prism_;
	pcl::EuclideanClusterExtraction<PointT> cluster_;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    pcl::PointCloud<PointT>::Ptr 	cloud_pt (new pcl::PointCloud<PointT>),
											input  (new pcl::PointCloud<PointT>),
											nan_input  (new pcl::PointCloud<PointT>),
											cloud_filtered (new pcl::PointCloud<PointT>),
											cloud_hull (new pcl::PointCloud<PointT>),
											cloud_projected (new pcl::PointCloud<PointT>);

	if(argc<=1){
		cout <<endl<<"please provide a pcd-file for further processing"<<endl<<endl;
		return 0;
	}

    //read in the sample cloud
	std::stringstream pcd;
	pcd<<argv[1];
	pcl::PCDReader reader;
	reader.read((pcd.str()),*nan_input);


	// remove NaN values and uninteresting data from Point cloud
	pcl::PassThrough<PointT> pass_x;
	pcl::PassThrough<PointT> pass_y;
	pcl::PassThrough<PointT> pass_z;
	pass_x.setInputCloud(nan_input);
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits(filter_min,filter_max);
	pass_x.filter(*cloud_pt);
	pass_y.setInputCloud(cloud_pt);
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits(filter_y_min,filter_y_max);
	pass_y.filter(*cloud_pt);
	pass_z.setInputCloud(cloud_pt);
	pass_z.setFilterFieldName("z");
	pass_z.setFilterLimits(filter_min,filter_max);
	pass_z.filter(*cloud_pt);


	// Create the normal estimation class, and pass the input dataset to it
	ne_.setInputCloud (cloud_pt);
	ne_.setSearchMethod (tree);
	ne_.setKSearch (k_search);
	ne_.compute (*cloud_normals);
	ne_.setViewPoint(0.0,0.0,1.5);


	//Segment the biggest plane (perpendicular to z-axis of base_link)
    pcl::ModelCoefficients::Ptr table_coeff (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices ());
    Eigen::Vector3f seg_axis(0.0,0.0,1.0);
    seg_.setInputCloud (cloud_pt);
    seg_.setInputNormals (cloud_normals);
	seg_.setOptimizeCoefficients (true);
	seg_.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg_.setMethodType (pcl::SAC_RANSAC);
	seg_.setDistanceThreshold (seg_distance_threshold);
	seg_.setNormalDistanceWeight (seg_normal_distance_weight);
	seg_.setEpsAngle(pcl::deg2rad(eps_angle));
	seg_.setMaxIterations (seg_iter);
	seg_.setProbability (seg_probability);
	seg_.segment (*table_inliers, *table_coeff);

    //Extract the biggest cluster corresponding to above inliers
    std::vector<pcl::PointIndices> tablevec;
    cluster_.setInputCloud (cloud_pt);
    cluster_.setIndices(table_inliers);
    cluster_.extract (tablevec);
    pcl::PointCloud<PointT>::Ptr 	tableplane (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud (*cloud_pt, tablevec, *tableplane);

    //projection of the table to a perfect plane
    proj_.setInputCloud (tableplane);
    proj_.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    proj_.setModelCoefficients (table_coeff);
    proj_.filter (*cloud_projected);


    // Create a Convex Hull representation of the projected inliers
    convhull_.setInputCloud (cloud_projected);
    convhull_.reconstruct (*cloud_hull);

    // Extract the object clusters using a polygonal prism and create a new cloud with the extracted objects
    pcl::PointIndices::Ptr objects_indices (new pcl::PointIndices ());
    prism_.setHeightLimits (cluster_min_height, cluster_max_height);
    prism_.setInputCloud (cloud_pt);
    prism_.setInputPlanarHull (cloud_hull);
    prism_.segment (*objects_indices);
    pcl::PointCloud<PointT>::Ptr extracted_objects (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud_pt,*objects_indices,*extracted_objects);

    //save extracted objects
    pcl::PCDWriter writer;
    writer.write ("extracted_"+pcd.str(), *extracted_objects, true);

    //compute centroid of the objects
    Eigen::Vector4f centroid_objects, centroid_table;
    pcl::compute3DCentroid(*extracted_objects,centroid_objects);
    pcl::compute3DCentroid(*tableplane,centroid_table);


    //build up of Workspace cloud
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr workspace_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    workspace_cloud = create_workspace(centroid_objects, extracted_objects, *table_coeff);


    //visualizing the cloud plus camera workspace
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = Vis(extracted_objects, centroid_objects, centroid_table,*table_coeff, workspace_cloud);

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


    return 0;


}
