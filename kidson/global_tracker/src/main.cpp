#include <iostream>
#include <algorithm>


#include "ground_truth.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <frame_common/stereo.h>
#include <image_geometry/stereo_camera_model.h>

#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <tf/transform_listener.h>
#include <kidnapped_robot/SavePlace.h>
#include <kidnapped_robot/MatchRequest.h>
#include <kidnapped_robot/MatchResult.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <pcl_ros/transforms.h>
#include <pcl/ros/conversions.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>

#include <kidnapped_robot/place_database.h>

#include <posest/pe3d.h>
#include "node_definitions.h"
// #include "optimizer.h"
#include <algorithm>

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include <cmath>

// optimizer stuff
//#include <g2o/core/graph_optimizer_sparse.h>
//#include <g2o/solvers/csparse/linear_solver_csparse.h>
//#include <g2o/solvers/pcg/linear_solver_pcg.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/linear_solver.h>
//
//#include <g2o/types/slam3d/vertex_depthcam.h> // definition of camera (pose & intrinsic)
//#include <g2o/types/slam3d/vertex_trackxyz.h> // definition of landmark
//
//#include <g2o/types/slam3d/edge_project_disparity.h> // oberservation of landmarl (cam <-> lm)
//#include <g2o/types/slam3d/edge_dc2dc.h>  // relative between cams (e.g. odo:  cam <-> cam)
#include <visualization_msgs/Marker.h>


#include "FrameGenerator.h"
//#include "optimizer.h"
#include "visualization.h"
#include "cluster_manager.h"

FrameGenerator* frame_generator;

// true iff next frame should be captured
bool getFrame = false;

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    signalMessage(msg);
  }
};

ofstream path;

// collection of all landmarks
map<int,Landmark> landmarks;

uint image_cnt;

using namespace sensor_msgs;
using namespace message_filters;
namespace enc = sensor_msgs::image_encodings;

kidnapped_robot::PlaceDatabase place_db_;
frame_common::FrameProc frame_processor_;
sensor_msgs::CvBridge cv_bridge_;

map<int,ros::Publisher> publishers;

typedef sync_policies::ApproximateTime<Image, stereo_msgs::DisparityImage, CameraInfo> MySyncPolicy;
typedef cv::CalonderDescriptorExtractor<float> Calonder;
ros::Publisher img_desc_pub;

pe::PoseEstimator3d* pose_estimator_;

uint start_skip = 0;
ros::NodeHandle* nh;

frame_common::Frame last_frame;
vector<cv::Vec3f> poses;

bool cam_params_set;

//ros::Publisher pub_init, pub_gt, pub_opt;

ros::Publisher node_pub;


ros::Publisher marker_pub;
ros::Publisher marker_pub_array;

vector<cv::Point3f> odo_path;
ros::Time last_odo_time;

// parameters can be changed via the slam.launch - file
int P_RANSAC_ITERATIONS = 10000;
bool P_DO_POLISH = false;
bool create_pointcloud = false;
string bag_file_name = "/home/engelhar/Desktop/groundtruth/freiburg1/rgbd_dataset_freiburg1_desk.bag";
bool do_live_data = false;
int min_strength = 20;
int max_point_cnt = 500;

int queue_size = 15;
int min_inlier_cnt = 12;
int tree_neighbour_cnt = 10;
int tree_skip_cnt = 10;
int search_depth = 1;
double min_dist_cm = 10;
double min_rot_deg = 5;
int last_match_ring_size = 10;
int last_nodes_cnt = 5;
double max_ransac_px_dev = 2;
double max_ransac_disp_dev = 0.5;
int max_node_id = -1;


bool do_gt_evaluation = true;
string gt_file_name = "/home/engelhar/Desktop/klt_track_desk.txt";

int img_file_id;

point_cloud complete_cloud;

Cluster_manager cluster_manager;



#define MARKER_SHOW

int marker_show_skip = 10;

#define POSE_COMPARISON





using namespace cluster_optimization;


int removeDuplicates(vector<cv::DMatch>& v)
{
  ROS_FATAL("tree_demo l. 349");
  assert(1==0);

  // lt_cv_cmatch_query lt_query;
  // lt_cv_cmatch_train lt_train;
  // vector<cv::DMatch>::iterator new_end;
  //
  // uint size = v.size();
  //
  // sort(v.begin(),v.end(), lt_query);
  // new_end = unique(v.begin(), v.end(),unique_dmatch);
  //
  //
  // sort(v.begin(), new_end, lt_train);
  // new_end =  unique(v.begin(), new_end,unique_dmatch);
  // v.resize(new_end-v.begin());
  //
  //
  // return size-v.size();
}



void imageCb_gt(const ImageConstPtr& img_rgb, const ImageConstPtr& img_depth, const sensor_msgs::CameraInfoConstPtr& info_msg){



  cout << "looking for pose at t = " << img_rgb->header.stamp << endl;
  stamped_trafo trafo;
  // false if no trafo with this stamp (+- 0.1s) was found
  if (!frame_generator->ground_truth.getTrafoAt(img_rgb->header.stamp, trafo)){
    return;
  }


  Node new_node;

  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(info_msg);

  frame_generator->createNode(new_node, img_rgb, img_depth, cam_model);

  //  cout << "found pose at t = " << trafo.time << endl;


  // adds transformed points to cloud
  Cluster_manager::getTransformed(new_node.frame.dense_pointcloud,trafo.trafo,complete_cloud );
  cout << "cloud has now " << complete_cloud.points.size() << " points" << endl;

}



void imageCb_depth(const ImageConstPtr& img_rgb, const ImageConstPtr& img_depth, const sensor_msgs::CameraInfoConstPtr& info_msg){



  // ros::Time cb_start = ros::Time::now();
  Node new_node;

  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(info_msg);

  frame_generator->createNode(new_node, img_rgb, img_depth, cam_model);

//  ROS_INFO("created node with id %i", new_node.node_id);

  new_node.odo_id = odo_path.size();


#ifdef POSE_COMPARISON


  cluster_manager.storeNode(new_node);
//  ROS_INFO("added node %i", new_node.node_id);
  cluster_manager.nodes[new_node.node_id] = new_node;
  return;

#endif


  //ros::Time e = ros::Time::now();

  // ROS_INFO("frame creation: %i ms", (e-cb_start).nsec/1000/1000);

#ifdef MARKER_SHOW

  if ((new_node.node_id % marker_show_skip) == 0)
  {
    ROS_INFO("trying to send %i", new_node.node_id);

    sensor_msgs::PointCloud2 msg;
    point_cloud foo;

    if(!new_node.ground_truth_valid){
      ROS_INFO("NO POSE INFORMATION");
      return;
    }

    ROS_INFO("sending pointcloud with %i points", (int) new_node.frame.dense_pointcloud.points.size());
    Cluster_manager::getTransformed(new_node.frame.dense_pointcloud, new_node.ground_truth_pose, foo);

    pcl::toROSMsg (foo,  msg);

    msg.header.frame_id="/fixed_frame";
    msg.header.stamp = ros::Time::now();
    node_pub.publish(msg);

  }
  else
  {
    ROS_INFO("skipping %i", new_node.node_id);
    //    ROS_INFO("no one listenes to node_pub");
  }
  return;
#endif



  // char key =  cv::waitKey(10);
  /*
 getFrame = (key == 110); // 'n'

 if (!getFrame)
  return;
 else
  getFrame = false;
   */


  // ros::Time add_start = ros::Time::now();
  set<uint> cluster_set = cluster_manager.addNode_tree_n_last(new_node);

  vector<uint> merged_clusters(cluster_set.begin(), cluster_set.end());


  ROS_INFO("merged_clusters.size() == %i", int(merged_clusters.size()));


  // optimize and merge clusters
  if (merged_clusters.size() > 1){

    uint added_node_cnt = 0;
    for (uint i=1; i<merged_clusters.size(); ++i){
      cluster* c = &cluster_manager.clusters[merged_clusters[i]];
      added_node_cnt += c->node_ids.size();
    }


    if (added_node_cnt < 10){
      ROS_ERROR("CLUSTER MERGE, But only %i pose in deleted clusters", (int) added_node_cnt);
    }
    else{


      // TODO: no merge if the other cluster only contains one node

      ROS_ERROR("CLUSTER MERGE");
      ROS_ERROR("CLUSTER MERGE");
      ROS_ERROR("CLUSTER MERGE");
      ROS_ERROR("CLUSTER MERGE");
      ROS_ERROR("CLUSTER MERGE");
      ROS_ERROR("CLUSTER MERGE");


      // all nodes were collected in the first cluster in the set.
      // this cluster is optimized to have a first alignment of the different clusters
      // so that we can find close cameras
      uint super_cluster_id = merged_clusters[0];
      cluster* merge_cluster = &cluster_manager.clusters[super_cluster_id];

      ROS_INFO("Optimizing supercluster %u", super_cluster_id);
      // cluster-pose is intialized with constant vel model
      optimize_cluster(cluster_manager, merge_cluster);

      for (uint i=1; i<merged_clusters.size(); i++){
        ROS_INFO("Searching for new node-combinations between clusters %i and %i", int(super_cluster_id), int(merged_clusters[i]));

        int hits = cluster_manager.mergeClusters(*merge_cluster, cluster_manager.clusters[merged_clusters[i]]);
        ROS_INFO("Found %i new connections", hits);
      }


      // and once again for fun
      // optimize_cluster(cluster_manager, merge_cluster);
    }




    // remove merged clusters
    for (uint i=1; i<merged_clusters.size(); ++i){
      ROS_INFO("finally removing merged cluster %i", merged_clusters[i]);
      cluster_manager.clusters.erase(merged_clusters[i]);
    }


  }

  if (cluster_manager.nodes.size() % 50 == 0){
    ROS_INFO("optimizer with N = %i", (int)cluster_manager.nodes.size());

    optimize_with_landmarks(cluster_manager, 15);

    //    if (create_pointcloud && pub_opt.getNumSubscribers () > 0)
    //      cluster_manager.sendClouds(pub_opt, PT_optimized,"/fixed_frame");

    char file[300];

    sprintf(file,"/home/engelhar/Desktop/current_run_output/all_nodes_%i.txt", int(cluster_manager.nodes.size()) );


    cluster_manager.writePosesToFile("/home/engelhar/Desktop/current_run_output/all_nodes.txt", false);
    cluster_manager.printTiming();
    frame_generator->printTiming();

    if (do_gt_evaluation)
      cluster_optimization::optimize_with_tracker(cluster_manager, 50);
  }


  ROS_INFO("%i cluster: ", (int) cluster_manager.clusters.size());
  uint nodes_in_large_clusters = 0;
  for (clm_it it = cluster_manager.clusters.begin(); it != cluster_manager.clusters.end(); ++it){
    if (it->second.node_ids.size() > 5){
      ROS_INFO("cl %i has %i nodes (%.0f %%)", int(it->first), int(it->second.node_ids.size()), ((it->second.node_ids.size())*100.0/cluster_manager.nodes.size()));
      nodes_in_large_clusters+=it->second.node_ids.size();
    }
  }
  ROS_INFO("%.0f %% of nodes are in large clusters",nodes_in_large_clusters*100.0/cluster_manager.nodes.size());


  //  ROS_INFO("callback finished");
  //  visualizeOdometry_GT(cluster_manager.nodes, *frame_generator, true); // estimate
  //  visualizeOdometry_GT(cluster_manager.nodes, *frame_generator, false); // gt



  // ros::Time add_end = ros::Time::now();

  // ROS_INFO("adding: %i s", (add_end-add_start).nsec/1000/1000);

  //if (cluster_manager.clusters.size() == 2){
  // optimize_cam_edges(cluster_manager.nodes);
  // if (marker_pub.getNumSubscribers() > 0)
  //  sendMarkers(marker_pub,marker_pub_array, cluster_manager.nodes);

  /*
 sendClouds(*nh, cluster_manager.nodes, publishers);
 sendClouds_simple(pub_pc2, cluster_manager.nodes);
   */

}


void visualizeOdometry(node_map& nodes)
{
  int size_px = 800;
  float border_m = 0.25; // size of border around bounding box
  float grid_dist = 0.25; // distance in m between gridlines

  cv::Mat odo_img(size_px, size_px,CV_32FC3);
  odo_img.setTo(cv::Scalar::all(0));

  // compute the track's bounding box
  cv::Point3f min_(1e5,1e5,0);
  cv::Point3f max_(-1e5,-1e5,0);


  for (uint i=0; i<odo_path.size()-1; i++)
  {
    cv::Point3f p = odo_path.at(i);
    // cout << "odo: "  << p.x << "  " << p.y << endl;
    min_.x = min(min_.x,p.x); max_.x = max(max_.x,p.x);
    min_.y = min(min_.y,p.y); max_.y = max(max_.y,p.y);
  }

  // 1m Abstand um die Bounding Box
  double m2px_x = (2*border_m+ max_.x-min_.x)*1.0/size_px;
  double m2px_y = (2*border_m+ max_.y-min_.y)*1.0/size_px;


  double m2px = max(m2px_x,m2px_y);

  vector<cv::Point2i> px_pos;
  for (uint i=0; i<odo_path.size(); i++)
  {
    cv::Point3f p = odo_path.at(i);
    px_pos.push_back( cv::Point2i( (border_m+(p.x-min_.x))/m2px,
                                   (border_m+(p.y-min_.y))/m2px) );
  }

  // draw grid to show scale:
  for (float x = (border_m+(0-min_.x)-20*grid_dist)/m2px; x <= size_px; x += grid_dist/m2px)
    cv::line(odo_img,cv::Point2i(x,0),cv::Point2i(x,size_px), cv::Scalar(255,255,255),1);

  for (float y = (border_m+(0-min_.y)-20*grid_dist)/m2px; y <= size_px; y += grid_dist/m2px)
    cv::line(odo_img,cv::Point2i(0,y),cv::Point2i(size_px,y), cv::Scalar(255,255,255),1);

  // draw the track
  // for (uint i=0; i<odo_path.size()-1; i++)
  //  cv::line(odo_img,px_pos[i],px_pos[i+1],cv::Scalar(0,0,255),1);


  // show current orientation:
  float phi = odo_path[odo_path.size()-1].z;
  float l = 0.10/m2px; // richtungspfeil ist 10cm lang
  float dx = cos(phi)*l;
  float dy = sin(phi)*l;

  cv::Point2i current = px_pos[px_pos.size()-1];
  cv::line(odo_img,current,current+cv::Point2i(dx,dy),cv::Scalar(0,255,0),1);


  // show node positions
  for (node_it it = nodes.begin(); it != nodes.end(); ++it)
  {
    Node* current = &(*it).second;
    if (current->is_keyframe)
      cv::circle(odo_img,px_pos[current->odo_id],3,cv::Scalar(255,0,0),1 );
    else
      cv::circle(odo_img,px_pos[current->odo_id],1,cv::Scalar(255,255,255),1 );
  }

  // show tree proposals for last node
  if (nodes.size() > 0){
    Node* last = &nodes.rbegin()->second;
    for (uint i=0; i<last->tree_proposals.size(); ++i)
      cv::line(odo_img,px_pos[last->odo_id],px_pos[nodes[last->tree_proposals[i]].odo_id],cv::Scalar::all(255),1);
  }

  // show edges:
  for (node_it it = nodes.begin(); it != nodes.end(); ++it)
  {
    Node* current = &(*it).second;



    for (uint j=0; j<current->matches.size(); j++)
    {
      Node_match* nm = &current->matches.at(j);

      if (current->node_id < nm->other_id)
        continue;

      // ROS_INFO("edges: %i %i", current->node_id, nm->other_id);
      Node* neighbour = &nodes[nm->other_id];
      //ROS_INFO("odo: %i has neighbour: %i",it->second.node_id,current.matches.at(j).other_id);

      /*
   // distance betweens nodes given by posest
   double cam_dist = current.dists[j];


   double dx = odo_path.at(current.odo_id).x-odo_path.at(neighbour.odo_id).x;
   double dy = odo_path.at(current.odo_id).y-odo_path.at(neighbour.odo_id).y;

   double odo_dist = sqrt(dx*dx+dy*dy);

   ROS_INFO("cam: %f odo: %f abs: %f", cam_dist,odo_dist, abs(cam_dist-odo_dist));

   cv::Scalar col;

   if (odo_dist == 0)
    col = cv::Scalar(0,255,255); // gelb bei fehlender odometrie
   else
    if (odo_dist > cam_dist)
    {
     // 1 < 1/b < (!) 2 // errors > factor 2 are full color
     col = cv::Scalar((odo_dist/cam_dist)/2*255,0,0);
    } else
    {
     col = cv::Scalar(0,(cam_dist/odo_dist)/2*255,0);
    }

       */

      int r,g,b;
      r=g=b=0;

      //  ROS_INFO("type: %i", nm->type);

      switch (nm->type) {
        case ET_Direct_neighbour:
          r = b = 255; break; // yellow
        case ET_Tree_Proposal:
          r = 255; break;     // red
        case ET_Ring:
          g = 255; break;
        case ET_Last_match:
          g = r = 255; break;
        default:
          r=g=b=255; break;
      }
      cv::Scalar col = cv::Scalar(b,g,r);
      cv::line(odo_img,px_pos[current->odo_id],px_pos[neighbour->odo_id],col,1);
    }
  }



  if (frame_generator->next_node_id % 10 == 0){
    char buffer[300];
    sprintf(buffer, "/home/engelhar/Desktop/img/%i.png", (int)frame_generator->next_node_id);
    // cout << "writing to " << buffer << endl;
    cv::imwrite(buffer, odo_img);
  }

  cv::imshow("odometry",odo_img);
  cv::waitKey(10);

}

void odometryCb(const nav_msgs::OdometryConstPtr& msg)
{

  return;

  if (odo_path.size() == 0)
  {
    // ROS_INFO("First Odometry-callback");
    last_odo_time = ros::Time::now();
    odo_path.push_back(cv::Point3f(0,0,0));
    return;
  }

  // ROS_INFO("Odometry: x: %f phi: %f",msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  ros::Time current_time = msg->header.stamp;

  // last position
  cv::Point3f pos = odo_path.at(odo_path.size()-1);

  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y; // currently always zero (for two-wheeled pioneer)
  double d_phi = msg->twist.twist.angular.z;

  double odo_phi = pos.z;

  double dt = (current_time - last_odo_time).toSec();
  double delta_x = (vx * cos(odo_phi) - vy * sin(odo_phi)) * dt;
  double delta_y = (vx * sin(odo_phi) + vy * cos(odo_phi)) * dt;
  double delta_phi = d_phi * dt;

  last_odo_time = current_time;

  cv::Point3f new_pos = pos+cv::Point3f(delta_x, delta_y, delta_phi);
  odo_path.push_back(new_pos);


  visualizeOdometry(cluster_manager.nodes);
}


int  main (int argc, char** argv)
{


  img_file_id = 0;
  // cv::namedWindow("odometry");
  ros::init(argc, argv, "tree_test_node");

  nh = new ros::NodeHandle();

  // cv::namedWindow("RGB");
  image_cnt = 0;
  cam_params_set = false;



  // path.open("/home/engelhar/Desktop/current_run_output/path.txt");


  ros::param::get("/MIN_INLIER_CNT", min_inlier_cnt);
  ros::param::get("/CREATE_POINTCLOUD", create_pointcloud);
  ros::param::get("/RANSAC_ITERATIONS", P_RANSAC_ITERATIONS);
  ros::param::get("/BAG_FILE_NAME", bag_file_name);
  ros::param::get("/DO_LIVE_DATA",do_live_data);
  ros::param::get("/MIN_INTEREST_POINT_STRENGTH",min_strength);
  ros::param::get("/NODE_QUEUE_SIZE", queue_size);
  ros::param::get("/TREE_NEIGHBOUR_CNT", tree_neighbour_cnt);
  ros::param::get("/MIN_INLIER_CNT", min_inlier_cnt);
  ros::param::get("/TREE_SKIP", tree_skip_cnt);
  ros::param::get("/NEIGHBOUR_SEARCH_DEPTH",search_depth);
  ros::param::get("/MIN_KEYFRAME_DIST_CM",min_dist_cm);
  ros::param::get("/MIN_KEYFRAME_ROT_DEG",min_rot_deg);
  ros::param::get("/MAX_INTEREST_POINT_CNT",max_point_cnt);
  ros::param::get("/LAST_MATCH_RING_SIZE",last_match_ring_size);
  ros::param::get("/LAST_NODE_CNT",last_nodes_cnt);

  ros::param::get("/GROUND_TRUTH_FILE_NAME",gt_file_name);
  ros::param::get("/EVAL_WITH_GROUND_TRUTH",do_gt_evaluation);


  ros::param::get("/MAX_PX_DEVIATION",max_ransac_px_dev);
  ros::param::get("/MAX_DISP_DEVIATION",max_ransac_disp_dev);
  ros::param::get("/MAX_NODE_NR",max_node_id);


  if (do_gt_evaluation){
    ROS_ERROR("Loading Tracker-Path from %s",gt_file_name.c_str());
  }else
    ROS_ERROR("No Track-Path given");


  if (max_node_id>0)
    ROS_ERROR("processing the first %i nodes", max_node_id);
  ROS_ERROR("RANSAC: inlier: < %.1f px and %.1f disp", max_ransac_px_dev,max_ransac_disp_dev);
  ROS_ERROR("Comparing with the last %i matches", last_match_ring_size);
  ROS_ERROR("Comparing with last %i added nodes", last_nodes_cnt);

  ROS_ERROR("Min dist for keyframe: %.0f cm or %.0f deg",min_dist_cm, min_rot_deg);
  ROS_ERROR("Using %i-Ring for every important hit", search_depth);
  // ROS_ERROR("Adding every %i.th node to tree",tree_skip_cnt );
  ROS_ERROR("Tree neighbour cnt: %i",tree_neighbour_cnt );
  ROS_ERROR("Min #inlier: %i",min_inlier_cnt );
  ROS_ERROR("Queue Size: %i", queue_size);
  ROS_ERROR("Min Interest Point strength: %i", min_strength);
  ROS_ERROR("Max Interest Point count: %i", max_point_cnt);
  ROS_ERROR("Ransac Iterations: %i", P_RANSAC_ITERATIONS);

  // min disp, max_dist_px, max_dist_dist
  pose_estimator_ = new  pe::PoseEstimator3d(P_RANSAC_ITERATIONS, P_DO_POLISH, 0, max_ransac_px_dev,max_ransac_disp_dev); // 2 .5

  //  pub_init = nh->advertise<sensor_msgs::PointCloud2>("/pose_init",5);
  //  pub_gt = nh->advertise<sensor_msgs::PointCloud2>("/pose_gt",5);
  //  pub_opt = nh->advertise<sensor_msgs::PointCloud2>("/pose_opt",5);
  node_pub = nh->advertise<sensor_msgs::PointCloud2>("/nodes",5);


  // marker_pub_array = nh->advertise<visualization_msgs::MarkerArray>( "/cam_position_array", 0 ); // Queue_size?
  // marker_pub = nh->advertise<visualization_msgs::Marker>( "/cam_edges", 0 ); // Queue_size?


  //  ROS_INFO("Listening to /camera/rgb/image_color /camera/depth/image /camera/rgb/camera_info (and /pose)");

  // TODO: add to launch-file
//  place_db_ = kidnapped_robot::PlaceDatabase("places.db","/opt/ros/diamondback/stacks/vslam/vocabulary_tree/holidays.tree",
//                                             "/opt/ros/diamondback/stacks/vslam/vocabulary_tree/holidays.weights");

  //place_db_ = kidnapped_robot::PlaceDatabase("places.db","/home/engelhar/Desktop/odu_files/images.tree",
  //                                           "/home/engelhar/Desktop/odu_files/images.weights");







  frame_generator = new FrameGenerator("/opt/ros/diamondback/stacks/vslam/vslam_system/data/calonder.rtc",
                                       create_pointcloud,min_strength,max_point_cnt);
  frame_generator->pub = nh->advertise<sensor_msgs::PointCloud2>("/frame_gen",5);

  frame_generator->has_groundtruth = do_gt_evaluation;
  if (do_gt_evaluation){
    cout << "loading gt" << endl;
    frame_generator->ground_truth = Ground_truth(gt_file_name);
    cout << "gt loading finished" << endl;
  }

  // start cluster manager:
  cluster_manager = Cluster_manager(queue_size,tree_neighbour_cnt,min_inlier_cnt,last_match_ring_size);
  cluster_manager.pose_estimator_ = pose_estimator_;
  cluster_manager.vt_db = frame_generator->vt_db;
  cluster_manager.search_depth = search_depth;

  ofstream off; off.open("/home/engelhar/Desktop/current_run_output/matches.txt");
  cluster_manager.match_stream = &off;

  ofstream off2; off2.open("/home/engelhar/Desktop/current_run_output/point_errors.txt");
  cluster_manager.point_error_stream = &off2;

  ofstream off3; off3.open("/home/engelhar/Desktop/current_run_output/all_comps.txt");
  cluster_manager.all_comparisons = &off3;




  cluster_manager.min_rot_deg = min_rot_deg;
  cluster_manager.min_dist_cm = min_dist_cm;
  cluster_manager.last_nodes_cnt = last_nodes_cnt;

  cluster_manager.pub_new = nh->advertise<sensor_msgs::PointCloud2>( "/tree/new_cloud", 1 );
  cluster_manager.pub_old = nh->advertise<sensor_msgs::PointCloud2>( "/tree/old_cloud", 1 );
  cluster_manager.pub_new_moved = nh->advertise<sensor_msgs::PointCloud2>( "/tree/new_cloud_moved", 1 );
  cluster_manager.pub_marker_array = nh->advertise<visualization_msgs::MarkerArray>("/inliers_array", 1 );


  //#define RECONSTRUCT_CLOUD


  if (do_live_data)
  {
    uint queue_size = 5;
    message_filters::Subscriber<Image> sub_rgb_img(*nh,"/camera/rgb/image_color", queue_size);
    //message_filters::Subscriber<stereo_msgs::DisparityImage> sub_disp(*nh,"/camera/depth/disparity", queue_size);
    message_filters::Subscriber<Image> sub_depth_img(*nh,"/camera/depth/image", queue_size);

    message_filters::Subscriber<CameraInfo> sub_rgb_info(*nh,"/camera/rgb/camera_info", queue_size);
    ros::Subscriber sub_odo = nh->subscribe("/pose", queue_size, odometryCb);

    typedef sync_policies::ApproximateTime<Image,Image, CameraInfo> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(queue_size), sub_rgb_img, sub_depth_img,sub_rgb_info);
    sync.registerCallback(boost::bind(&imageCb_depth, _1, _2,_3));

    ros::spin();
  }
  else
  {

    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/rgb/image_color"));
    topics.push_back(std::string("/camera/depth/image"));
    topics.push_back(std::string("/camera/rgb/camera_info"));
    topics.push_back(std::string("/pose"));

    BagSubscriber<sensor_msgs::Image> sub_col;
    BagSubscriber<sensor_msgs::Image> sub_depth;
    BagSubscriber<CameraInfo> sub_info;
    BagSubscriber<nav_msgs::Odometry> sub_odo;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
        sensor_msgs::Image,
        CameraInfo> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(25,sub_col, sub_depth,sub_info);

#ifdef RECONSTRUCT_CLOUD
    sync.registerCallback(boost::bind(&imageCb_gt, _1, _2, _3));
    complete_cloud.points.clear();
#else
    sync.registerCallback(boost::bind(&imageCb_depth, _1, _2, _3));
#endif

    rosbag::Bag bag;

    ROS_ERROR("Trying to open bagfile at %s", bag_file_name.c_str());
    // TODO: catch exception if file is not found
    bag.open(bag_file_name.c_str(), rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(topics));


    int64 bag_start = cv::getTickCount();


    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {

      if (m.getTopic() == topics[0]) { sub_col.newMessage(m.instantiate<sensor_msgs::Image>()); }
      if (m.getTopic() == topics[1]) { sub_depth.newMessage(m.instantiate<sensor_msgs::Image>()); }
      if (m.getTopic() == topics[2]) { sub_info.newMessage(m.instantiate<CameraInfo>()); }
      if (m.getTopic() == topics[3]) {
        nav_msgs::OdometryConstPtr odo = m.instantiate<nav_msgs::Odometry>(); odometryCb(odo); }

      ros::spinOnce();

//      ROS_WARN("nodes.size() = %i", int(cluster_manager.nodes.size()));
      if (max_node_id > 0 && int(cluster_manager.nodes.size()) > max_node_id)
        break;
    }

    int64 bag_end = cv::getTickCount();


    //    ROS_INFO("start: %i, end %i, freq: %f", bag_start, bag_end, cv::getTickFrequency());
    double sec = (bag_end-bag_start)*1.0/cv::getTickFrequency();

    ROS_INFO("Bag completed"); bag.close();
    ROS_ERROR("processed Bag in %.1f s (%.3f s per Frame)", sec, sec*1.0/cluster_manager.nodes.size());


#ifdef POSE_COMPARISON

    ROS_WARN("Starting tree_check");

    uint ignored_last = 10;
    int tree_hits = 300;


    // compute incremental proposals
    cluster_manager.tree_check_incremental(ignored_last,tree_hits);
    ROS_WARN("incremental test finised");

    // add last nodes
    for ( uint i = cluster_manager.nodes.size()- ignored_last;  i < cluster_manager.nodes.size(); ++i){
      Node* add = &cluster_manager.nodes[i];
      int doc_id = frame_generator->vt_db->insert(add->words);
      cluster_manager.treeId2NodeId[doc_id] = add->node_id;
      ROS_ERROR("adding node %i to tree with docId %i", int(add->node_id), doc_id);
    }

    // and create batch proposals
    cluster_manager.tree_check_batch(ignored_last,tree_hits);
    ROS_WARN("Finished nicely");

//    ROS_INFO("starting n^2 comparison of %i nodes", cluster_manager.nodes.size());
//
//    uint hits = 0;
//    int inliers;
//    for (uint i=0; i<cluster_manager.nodes.size()-1; ++i){
//      ROS_INFO("checking node %i, so far %i hits", i, hits);
//      for (uint j=i+1; j<cluster_manager.nodes.size(); ++j){
//        bool hit = cluster_manager.check_pair(&cluster_manager.nodes[i], &cluster_manager.nodes[j], ET_Keyframe_check,inliers);
//        if (hit) hits++;
//      }
//    }
//
//
//    ROS_INFO("found %i hits", hits);

#endif



    // and once at the end

#ifndef RECONSTRUCT_CLOUD
#ifndef POSE_COMPARISON
    ROS_INFO("Last Optimization with N = %i", (int)cluster_manager.nodes.size());

    //    ROS_WARN("OPTIMIZING OF ALL CLUSTERS");
    //    optimize_with_landmarks(cluster_manager, 20);


    //    cluster_manager.ConsolidateCluster();

    //    for (clm_it it = cluster_manager.clusters.begin(); it != cluster_manager.clusters.end(); ++it){
    //      it->second.changed = true;
    //    }

    // many iterations in the final optimization!

    if (do_gt_evaluation)
      cluster_optimization::optimize_with_tracker(cluster_manager, 50);
    else
      cluster_optimization::optimize_with_landmarks(cluster_manager, 25);





    cluster_manager.printTiming();
    frame_generator->printTiming();

    ROS_ERROR("Last Optimization finished, Bag completely processed, Terminating");
    //    ROS_INFO("processed Bag in %i s (%f s per Frame)", processing_time.sec, processing_time.sec*1.0/cluster_manager.nodes.size());
#endif
#else
    // save Pointcloud

    point_cloud filtered;
    ROS_INFO("putting %i points in voxel grid", int(complete_cloud.points.size()));
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;

    sor.setInputCloud (complete_cloud.makeShared());
    sor.setLeafSize (0.01f, 0.01f, 0.01f);

    point_cloud cloud_filtered;
    sor.filter (cloud_filtered);

    pcl::PCDWriter writer;


    writer.write<point_type> ("/home/engelhar/Desktop/current_run_output/filtered_cloud.pcd", cloud_filtered, false);
    ROS_INFO("wrote %i points to file", int(cloud_filtered.points.size()));
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud_filtered, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/fixed_frame";
    cluster_manager.pub_new.publish(msg);


#endif

    //      cluster_manager.sendClouds(pub_opt, PT_optimized,"/fixed_frame");

  }

  // while(true)ros::spinOnce();;

  return 0;

}
