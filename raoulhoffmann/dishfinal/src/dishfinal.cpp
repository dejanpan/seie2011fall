#include <iostream>
#include <cstdlib>
#include <limits>
#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/point_cloud_conversion.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include "pcl_ros/io/bag_io.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl_ros/transforms.h>

#include <Eigen/Core>

class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      //nr_iterations_ (500)
      nr_iterations_ (2000)
    {
      // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};



inline void
matrixAsTransfrom (const Eigen::Matrix4f &out_mat,  tf::Transform& bt)
{
     double mv[12];

     mv[0] = out_mat (0, 0) ;
     mv[4] = out_mat (0, 1);
     mv[8] = out_mat (0, 2);
     mv[1] = out_mat (1, 0) ;
     mv[5] = out_mat (1, 1);
     mv[9] = out_mat (1, 2);
     mv[2] = out_mat (2, 0) ;
     mv[6] = out_mat (2, 1);
     mv[10] = out_mat (2, 2);

     btMatrix3x3 basis;
     basis.setFromOpenGLSubMatrix(mv);
     btVector3 origin(out_mat (0, 3),out_mat (1, 3),out_mat (2, 3));

     ROS_DEBUG("origin %f %f %f", origin.x(), origin.y(), origin.z());

     bt = tf::Transform(basis,origin);
}



// used for Pointcloud Messages, Debugging Purposes
ros::Publisher pubDebug;


// used for Pointcloud Messages, Debugging Purposes
ros::Publisher pubDebug2;
// used for Pointcloud Messages, Debugging Purposes
ros::Publisher pubBox;
// used for MarkerArrays, visualizing found cups
ros::Publisher pubMarkersCups;
// used for MarkerArrays, visualizing found plates
ros::Publisher pubMarkersPlates;

//save all known coordinates
std::vector<btVector3> coordsOfCups;
std::vector<btQuaternion> rotationsOfCups;

std::vector<btVector3> coordsOfPlates;
std::vector<btQuaternion> rotationsOfPlates;

//load templates
FeatureCloud cupNoHandleTemplate;
FeatureCloud cupHandleTemplate;
FeatureCloud plateTemplate;
//FeatureCloud plateArcTemplate; //unsure if necessary

TemplateAlignment templateAlign;
tf::TransformBroadcaster *br = 0;
void cloudThrottledCallback(const sensor_msgs::PointCloud2::ConstPtr& inputCloud)
{
    if (!br)
        br = new tf::TransformBroadcaster();
    double minX, minY, minZ, maxX, maxY, maxZ;
    ros::param::param<double>("D_minX", minX, 0.55);
    ros::param::param<double>("D_minY", minY, 0.28);
    ros::param::param<double>("D_minZ", minZ, 0.6);
    ros::param::param<double>("D_maxX", maxX, 0.96);
    ros::param::param<double>("D_maxY", maxY, 0.99);
    ros::param::param<double>("D_maxZ", maxZ, 0.77);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclInputCloud (new pcl::PointCloud<pcl::PointXYZ>);

    // convert pointcloud message to pcl pointcloud
    pcl::fromROSMsg(*inputCloud, *pclInputCloud);


    tf::StampedTransform transformOptToMap;
    tf::TransformListener listener;
    listener.waitForTransform("/openni_rgb_optical_frame", "/map", ros::Time(0), ros::Duration(10));
    listener.lookupTransform("/map", "/openni_rgb_optical_frame", ros::Time(0), transformOptToMap);
    pcl_ros::transformPointCloud(*pclInputCloud, *pclInputCloud ,transformOptToMap);

    Eigen::Vector4f min_pt, max_pt;

    min_pt = Eigen::Vector4f(minX,minY,minZ, 1);
    max_pt = Eigen::Vector4f(maxX,maxY,maxZ, 1);

     boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );
     pcl::getPointsInBox(*pclInputCloud,min_pt,max_pt,*indices);

     ROS_INFO("min %f %f %f" ,min_pt[0],min_pt[1],min_pt[2]);
     ROS_INFO("max %f %f %f" ,max_pt[0],max_pt[1],max_pt[2]);
     ROS_INFO("cloud size : %zu", pclInputCloud->points.size());

     pcl::ExtractIndices<pcl::PointXYZ> extr;
     extr.setInputCloud(pclInputCloud);
     extr.setIndices(indices);
         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
     extr.filter(*cloud);

     ROS_INFO("cloud size after box filtering: %zu", cloud->points.size());


    sensor_msgs::PointCloud2 outputBoxCloud;
    pcl::toROSMsg(*cloud, outputBoxCloud);
    outputBoxCloud.header.frame_id = "/map"; //inputCloud->header.frame_id;
    pubBox.publish(outputBoxCloud);




    // -------------------------------
    // STEP 1 : Euclidean Segmentation
    // -------------------------------
    std::vector<pcl::PointIndices> clusterIndices;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.005);
    ec.setMinClusterSize (1000);
    ec.setMaxClusterSize (50000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(cloud);

    ec.extract (clusterIndices);

    pcl::ExtractIndices<pcl::PointXYZ> ei;
    ei.setInputCloud(cloud);

    //iterate through found clusters
    unsigned int currentClusterIndex;
    pcl::PointCloud<pcl::PointXYZ>::Ptr    oneCluster         (new pcl::PointCloud<pcl::PointXYZ>);

    for (currentClusterIndex = 0; currentClusterIndex < clusterIndices.size();  currentClusterIndex++)
    {
        ei.setIndices(boost::make_shared<pcl::PointIndices>(clusterIndices[currentClusterIndex]));
        ei.filter(*oneCluster);

        // ---------------------------------------------
        // STEP 2 : fit the templates into each cluster
        // ---------------------------------------------

        FeatureCloud targetCloud;
        targetCloud.setInputCloud (oneCluster);
        templateAlign.setTargetCloud (targetCloud);

        TemplateAlignment::Result bestAlignment;
        int bestResultIndex = templateAlign.findBestAlignment (bestAlignment);
        if(bestAlignment.fitness_score > 0.000030)
        {
            //no good fit found
            printf ("Bad fit, score: %f\n", bestAlignment.fitness_score);

        }
        else
        {
            //good fit found
            Eigen::Matrix3f rotation = bestAlignment.final_transformation.block<3,3>(0, 0);
            Eigen::Vector3f translation = bestAlignment.final_transformation.block<3,1>(0, 3);

            tf::Transform mTransf;
            mTransf.setOrigin(tf::Vector3(0.05, 0.678, 0.575));
            mTransf.setRotation (tf::createQuaternionFromRPY(0, 0, 2.3));


            tf::Transform transfBestAlign;
            btMatrix3x3 rot;

            matrixAsTransfrom(bestAlignment.final_transformation, transfBestAlign);

            geometry_msgs::Pose posemsg;
            tf::poseTFToMsg(transfBestAlign, posemsg);
            std::cout << posemsg << std::endl;

            tf::Transform transfMapPlate;
            transfMapPlate.setOrigin(tf::Vector3(-0.045, -0.150, 0.773));
            transfMapPlate.setRotation (tf::Quaternion(0.667, -0.026, 0.715, 0.208));

            tf::Transform transfMapPlateGrip;
            transfMapPlateGrip.setOrigin(tf::Vector3(0.011, -0.099, 0.768));
            transfMapPlateGrip.setRotation (tf::Quaternion(0.191, -0.555, 0.809, 0.040));


            tf::Transform transfMapCup;
            transfMapCup.setOrigin(tf::Vector3(0.198, -0.014, 0.856));
            transfMapCup.setRotation (tf::Quaternion(0.399, -0.314, 0.793, 0.336));

            tf::Transform transfMapCupGrip;
            transfMapCupGrip.setOrigin(tf::Vector3(0.164, -0.085, 0.906));
            transfMapCupGrip.setRotation (tf::Quaternion(0.460, -0.093, 0.837, 0.280));


            //br->sendTransform(tf::StampedTransform(mTransf2, ros::Time::now(), "/map", "/dishfinal_template"));

            pcl::PointCloud<pcl::PointXYZ> transformedCloud;
            switch(bestResultIndex)
            {
            case 0:
                printf ("Cup without handle found! score: %f\n", bestAlignment.fitness_score);
                br->sendTransform(tf::StampedTransform(transfBestAlign*transfMapCup, ros::Time::now(), "/map", "/dishfinal_template"));
                br->sendTransform(tf::StampedTransform(transfBestAlign*transfMapCupGrip, ros::Time::now(), "/map", "/dishfinal_grip"));
                pcl::transformPointCloud (*cupNoHandleTemplate.getPointCloud (), transformedCloud, bestAlignment.final_transformation);
                  break;
            case 1:
                printf ("Cup with handle found! score: %f\n", bestAlignment.fitness_score);
                br->sendTransform(tf::StampedTransform(transfBestAlign*transfMapCup, ros::Time::now(), "/map", "/dishfinal_template"));
                br->sendTransform(tf::StampedTransform(transfBestAlign*transfMapCupGrip, ros::Time::now(), "/map", "/dishfinal_grip"));
                pcl::transformPointCloud (*cupHandleTemplate.getPointCloud (), transformedCloud, bestAlignment.final_transformation);
                  break;
            case 2:
                printf ("Plate found! score: %f\n", bestAlignment.fitness_score);
                br->sendTransform(tf::StampedTransform(transfBestAlign*transfMapPlate, ros::Time::now(), "/map", "/dishfinal_template"));
                br->sendTransform(tf::StampedTransform(transfBestAlign*transfMapPlateGrip, ros::Time::now(), "/map", "/dishfinal_grip"));
                pcl::transformPointCloud (*plateTemplate.getPointCloud (), transformedCloud, bestAlignment.final_transformation);
                  break;
            default:
                printf ("Something strange happened: Unknown index for template cloud.");  break;
            }

            sensor_msgs::PointCloud2 outputCloud;

            //render Templates
            pcl::toROSMsg(*plateTemplate.getPointCloud (), outputCloud);  outputCloud.header.frame_id = "/pcd_debug";
            pubDebug.publish(outputCloud);

            //render fitted Template
            pcl::toROSMsg(transformedCloud, outputCloud);  outputCloud.header.frame_id = "/map"; //inputCloud->header.frame_id;
            pubDebug2.publish(outputCloud);

            printf ("\n");
            printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
            printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
            printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
            printf ("\n");
            printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

        }



    }


}


int main(int argc, char **argv)
 {
 ros::init(argc, argv, "dishfinal");
 ros::NodeHandle n;
 ros::Subscriber sub = n.subscribe("/kinect/cloud_throttled", 2, cloudThrottledCallback);

 cupNoHandleTemplate.loadInputCloud ("cup_nohandle.pcd");
 cupHandleTemplate.loadInputCloud ("cup_handle.pcd");
 plateTemplate.loadInputCloud ("clean_plate.pcd");
 //plateArcTemplate.loadInputCloud ("plateArc.pcd"); //unsure if necessary

 templateAlign.addTemplateCloud (cupNoHandleTemplate);
 templateAlign.addTemplateCloud (cupHandleTemplate);
 templateAlign.addTemplateCloud (plateTemplate);
 //templateAlign.addTemplateCloud (plateArcTemplate); //unsure if necessary

 pubDebug = n.advertise<sensor_msgs::PointCloud2> ("dishfinal_debug", 1);
 pubDebug2 = n.advertise<sensor_msgs::PointCloud2> ("dishfinal_debug2", 1);
 pubBox = n.advertise<sensor_msgs::PointCloud2> ("dishfinal_box", 1);
 pubMarkersCups = n.advertise<visualization_msgs::MarkerArray>("dishfinal_cups", 0);
 pubMarkersPlates = n.advertise<visualization_msgs::MarkerArray>("dishfinal_plates", 0);

 ros::spin();
 }
