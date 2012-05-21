/*
 * visualization.cpp
 *
 *  Created on: Jul 24, 2011
 *      Author: engelhar
 */


#include "visualization.h"

//
//void readMatches(match_list& ml, string file){
//
//  fstream ifs; ifs.open(filename.c_str());
//  assert(ifs.is_open());
//
//  int n1, n2, cnt;
//
//  while (true){
//
//   ifs >> n1;
//   if (ifs.eof()) break;
//
//   ifs >> cnt;
//
//
//   ROS_INFO("reading %i edges for node ", cnt, n1);
//
//   vector<int> v(cnt);
//
//   for (int i=0; i<cnt; ++i){
//     ifs >> n2;
//     v.push_back(n2);
//   }
//
//   ml(n1) = v;
//
//}




void visualizeOdometry_GT(node_map& nodes, FrameGenerator& frame_gen, bool estimate)
{


  int size_px = 800;
  float border_m = 0.25; // size of border around bounding box

  cv::Mat odo_img(size_px, size_px,CV_32FC3);
  odo_img.setTo(cv::Scalar::all(0));

  // compute the track's bounding box
  cv::Point2f min_(1e5,1e5);
  cv::Point2f max_(-1e5,-1e5);


  for (node_it it = nodes.begin(); it != nodes.end(); ++it)
  {
    cv::Point2f p;

    if (estimate){
      p.x = (*it).second.opt_pose(0,3);
      p.y = (*it).second.opt_pose(1,3);
    }
    else
      p = (*it).second.gt_pos_xy;

    min_.x = min(min_.x,p.x); max_.x = max(max_.x,p.x);
    min_.y = min(min_.y,p.y); max_.y = max(max_.y,p.y);
  }

  // 0.5 Abstand um die Bounding Box
  double m2px_x = (2*border_m+ (max_.x-min_.x))/size_px;
  double m2px_y = (2*border_m+ (max_.y-min_.y))/size_px;



  double m2px = max(m2px_x,m2px_y);

  //  ROS_INFO("m2px: %f", m2px);
  //  ROS_INFO("min: %f %f", min_.x, min_.y);

  map<uint, cv::Point> px_pos;
  for (node_it it = nodes.begin(); it != nodes.end(); ++it)
  {
    cv::Point2f p;
    if (estimate){
      p.x = (*it).second.opt_pose(0,3);
      p.y = (*it).second.opt_pose(1,3);
    }
    else
      p = (*it).second.gt_pos_xy;
    //    printf("gt_pos: %f %f \n", p.x,p.y);
    px_pos[it->first] = cv::Point2i( (p.x-(min_.x-border_m))/m2px , (p.y-(min_.y-border_m))/m2px ) ;
    //    ROS_INFO("px: %i %i",px_pos[it->first].x, px_pos[it->first].y );
    // px_pos[it->first] = cv::Point2i( (border_m+(p.x-min_.x))/m2px,  (border_m+(p.y-min_.y))/m2px );
  }


  for (float x = floor(min_.x-border_m); x <= ceil(max_.x+border_m); x+=0.25) {
    int px_x = (x-(min_.x-border_m))/m2px;
    cv::line(odo_img,cv::Point2i(px_x,0),cv::Point2i(px_x,size_px), cv::Scalar(255,255,255),1);
  }

  for (float y = floor(min_.y-border_m); y <= ceil(max_.y+border_m); y+=0.25) {
    int px_y = (y-(min_.y-border_m))/m2px;
    cv::line(odo_img,cv::Point2i(0,px_y),cv::Point2i(size_px,px_y), cv::Scalar(255,255,255),1);
  }

  // show node positions
  for (node_it it = nodes.begin(); it != nodes.end(); ++it)
  {
    Node* current = &(*it).second;
    if (current->is_keyframe)
      cv::circle(odo_img,px_pos[current->node_id],3,cv::Scalar(255,0,0),1 );
    else
      cv::circle(odo_img,px_pos[current->node_id],1,cv::Scalar(255,255,255),1 );
  }



  //  if (nodes.size() > 0){
  //    Node* last = &nodes.rbegin()->second;
  //    for (uint i=0; i<last->tree_proposals.size(); ++i)
  //      cv::line(odo_img,px_pos[last->node_id],px_pos[nodes[last->tree_proposals[i]].node_id],cv::Scalar::all(255),1);
  //  }



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
      cv::line(odo_img,px_pos[current->node_id],px_pos[neighbour->node_id],col,1);
    }
  }



  //  if (frame_gen.node_id_running % 10 == 0){
  //    char buffer[300];
  //    sprintf(buffer, "/home/engelhar/Desktop/img/%i.png", (int)frame_gen.node_id_running);;
  //    cv::imwrite(buffer, odo_img);
  //  }


  if (estimate){
    cv::namedWindow("pos_Est",1);
    cv::imshow("pos_Est",odo_img);
  }
  else
  {
    cv::namedWindow("odometry_GT",1);
    cv::imshow("odometry_GT",odo_img);
  }

  cv::waitKey(10);


}

void sendClouds_simple(ros::Publisher pub, node_map& nodes)
{

  if (pub.getNumSubscribers() == 0)
    return;

  sensor_msgs::PointCloud2 msg;

  for (node_map::iterator node_it = nodes.begin(); node_it != nodes.end(); ++node_it){

    Node* node = &node_it->second;

    pcl::PointCloud<pcl::PointXYZRGB> pc_global;
    pcl::transformPointCloud(node->frame.dense_pointcloud,pc_global,node->pose.cast<float>());
    pcl::toROSMsg (pc_global,  msg);

    msg.header.frame_id= "/openni_rgb_optical_frame"; // /fixed_frame";
    msg.header.stamp = ros::Time::now();
    msg.header.seq = node->node_id;

    // ROS_INFO("sending cloud %i", node->node_id);
    pub.publish(msg);
  }
}


//void send



void sendMarkers(ros::Publisher pub_marker, ros::Publisher pub_marker_array , node_map& nodes)
{

  visualization_msgs::MarkerArray m_array;

  if (pub_marker_array.getNumSubscribers() > 0)
  {
    for (node_map::iterator nm_it = nodes.begin(); nm_it !=nodes.end(); ++nm_it)
    {
      visualization_msgs::Marker marker;

      visualization_msgs::Marker marker_text;

      Eigen::Vector4d P = Eigen::Vector4d::Zero();
      P.array()[3] = 1;

      Eigen::Vector4d start = nm_it->second.pose*P;
      P.array()[2] = 0.2; // 20cm in  z-direction
      Eigen::Vector4d end = nm_it->second.pose*P;

      // cerr << "node " << nm_it->second.node_id << " at: " << start << endl;

      marker.header.frame_id = "/openni_rgb_optical_frame";
      marker.header.stamp = ros::Time();
      marker.ns = "cam_positions_ns";
      marker.id = nm_it->first*2;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::MODIFY; // same as add



      geometry_msgs::Point p,p2;
      p.x = start.array()[0]; p.y = start.array()[1];  p.z = start.array()[2];
      marker.points.push_back(p);

      p2.x = end.array()[0]; p2.y = end.array()[1]; p2.z = end.array()[2];
      marker.points.push_back(p2);


      marker.scale.x = 0.01; // radius in m
      marker.scale.y = 0.03;  // head radius in m

      marker.color.a = 1;

      // ROS_INFO("drawing node with cluster %i ( mod 3 = %i)",nm_it->second.cluster_id, nm_it->second.cluster_id%3 );
      // cout << "at " << nm_it->second.pose << endl;

      marker.color.r = marker.color.g = marker.color.b = 0;

      switch (nm_it->second.cluster_id%3) {
        case 0: marker.color.b = 1; break;
        case 1: marker.color.g = 1; break;
        case 2: marker.color.r = 1; break;

        default:
          break;
      }

      m_array.markers.push_back(marker);

      marker_text.header.frame_id = "/openni_rgb_optical_frame";//"/fixed_frame";
      //marker_text.header.frame_id = "/fixed_frame";
      marker_text.header.stamp = ros::Time();
      marker_text.ns = "cam_positions_ns";
      marker_text.id = nm_it->first*2+1;
      marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker_text.action = visualization_msgs::Marker::MODIFY; // same as add
      char buff[20]; sprintf(buff, "%i", nm_it->first);

      marker_text.text = string(buff);
      marker_text.color = marker.color;
      marker_text.scale.z = 0.02;
      marker_text.pose.position.x = nm_it->second.pose(0,3);
      marker_text.pose.position.y = nm_it->second.pose(1,3);
      marker_text.pose.position.z = nm_it->second.pose(2,3)-0.1;


      m_array.markers.push_back(marker_text);


    }

    // ROS_ERROR("published %i arrows", (int) m_array.markers.size());
    pub_marker_array.publish(m_array);
  }

  if (pub_marker.getNumSubscribers() > 0)
  {

    // publish connections between cameras
    visualization_msgs::Marker m_cam_edge_list;
    m_cam_edge_list.header.frame_id = "/openni_rgb_optical_frame"; //"/fixed_frame";
    m_cam_edge_list.header.stamp = ros::Time();
    m_cam_edge_list.ns = "cam_matches_ns";
    m_cam_edge_list.id++;
    m_cam_edge_list.type = visualization_msgs::Marker::LINE_LIST;
    m_cam_edge_list.action = visualization_msgs::Marker::MODIFY;

    m_cam_edge_list.color.a =1;
    m_cam_edge_list.color.g = 1;
    m_cam_edge_list.color.r = m_cam_edge_list.color.b = 0;
    m_cam_edge_list.scale.x = 0.005; // line width in m

    for (node_map::iterator nm_it = nodes.begin(); nm_it !=nodes.end(); ++nm_it)
    {
      Node* current = &nm_it->second;
      Eigen::Vector4d P = Eigen::Vector4d::Zero();
      P.array()[3] = 1; // homogeneous coordinates

      Eigen::Vector4d p1 = current->pose*P;
      geometry_msgs::Point p; p.x = p1.array()[0]; p.y = p1.array()[1];  p.z = p1.array()[2];


      for (uint i=0; i<current->matches.size(); ++i)
      {
        Node* other = &nodes[current->matches.at(i).other_id];
        Eigen::Vector4d p2 = other->pose*P;
        geometry_msgs::Point gp; gp.x = p2.array()[0]; gp.y = p2.array()[1];  gp.z = p2.array()[2];

        // ROS_INFO("vis edge between %i and %i", current->node_id, other->node_id);

        m_cam_edge_list.points.push_back(p);
        m_cam_edge_list.points.push_back(gp);
      }
    }

    pub_marker.publish(m_cam_edge_list);
  }
}


void sendClouds(ros::NodeHandle& nh, node_map& nodes, map<int,ros::Publisher>& publishers)
{


  sensor_msgs::PointCloud2 msg;

  for (node_map::iterator node_it = nodes.begin(); node_it != nodes.end(); ++node_it){

    Node* node = &node_it->second;

    if (publishers.find(node->cluster_id) == publishers.end())
    {
      // create new publisher:
      ros::Publisher pub;
      char buff[20]; sprintf(buff, "/cluster_%i", node->cluster_id);
      pub  = nh.advertise<sensor_msgs::PointCloud2>(string(buff),5);

      publishers[node->cluster_id] = pub;
    }


    if (publishers.find(node->cluster_id)->second.getNumSubscribers() == 0)
      continue;

    pcl::PointCloud<pcl::PointXYZRGB> pc_global;
    pcl::transformPointCloud(node->frame.dense_pointcloud,pc_global,node->pose.cast<float>());
    pcl::toROSMsg (pc_global,  msg);

    msg.header.frame_id= "/openni_rgb_optical_frame"; // /fixed_frame";
    msg.header.stamp = ros::Time::now();
    msg.header.seq = node->node_id;

    publishers.find(node->cluster_id)->second.publish(msg);

  }



}

void drawColoredMatches(const Node& n1, const Node& n2, const vector<cv::DMatch>&matches,  cv::Mat& outImg){

  outImg = cv::Mat(480,640*2, CV_8UC3);

  //cv::Size size(640*2,480);
  //outImg.create( size, CV_32FC3 );
  cv::Mat outImg1 = outImg( cv::Rect(0, 0, 640, 480) );
  cv::Mat outImg2 = outImg( cv::Rect(640, 0, 640, 480) );

  n1.frame.img.copyTo(outImg1);
  n2.frame.img.copyTo(outImg2);


  for (uint i=0; i<matches.size(); ++i){
    cv::DMatch m = matches[i];

    cv::Scalar color;

    if (m.distance < 10)
      color = CV_RGB(0,0,255);
    else
      if (m.distance < 25)
        color = CV_RGB(255,255,0); // gelb
      else
        color = CV_RGB(255,0,0);

    cv::Point p1 = n1.frame.kpts[m.queryIdx].pt;
    cv::Point p2 = n2.frame.kpts[m.trainIdx].pt;

    cv::circle(outImg1, p1, 3, color, 1);
    cv::circle(outImg2, p2, 3, color, 1);
    cv::line(outImg, p1, cv::Point(p2.x+640,p2.y), color, 1);


  }

}
