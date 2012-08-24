/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "parameter_server.h"

using namespace std;

ParameterServer* ParameterServer::_instance = NULL;

ParameterServer::ParameterServer() {
    pre = ros::this_node::getName();
    pre += "/config/";

    defaultConfig();
    getValues();
    checkValues();
}

ParameterServer* ParameterServer::instance() {
    if (_instance == NULL) {
        _instance = new ParameterServer();
    }
    return _instance;
}
void ParameterServer::defaultConfig() {
    config["topic_image_mono"]              = std::string("/camera/rgb/image_color");	//record
    config["topic_image_depth"]             = std::string("/camera/depth/image");		//record
    config["topic_points"]                  = std::string("/camera/rgb/points"); 		//record
    config["wide_topic"]                    = std::string("/wide_stereo/left/image_mono");
    config["wide_cloud_topic"]              = std::string("/wide_stereo/points2");
    config["camera_info_topic"]             = std::string("/camera/rgb/camera_info");	//record
    config["individual_cloud_out_topic"]    = std::string("/rgbdslam/batch_clouds");
    config["aggregate_cloud_out_topic"]     = std::string("/rgbdslam/aggregate_clouds");
    config["fixed_frame_name"]              = std::string("/map");
    config["ground_truth_frame_name"]       = std::string("");                  //use empty string if no ground truth tf frame available
    config["base_frame_name"]               = std::string("/openni_camera");    //if the camera is articulated use robot base
    //config["bagfile_name"]                  = std::string("/work/kidson/bag_files/bench1-3sweeps3.bag");
    config["bagfile_name"]                  = std::string("/work/kidson/bag_files/radu_kitchen/karol_kitchen_2.bag");
    config["batch_processing"]              = static_cast<bool> (false);        //store results and close after bagfile has been processed
    config["fixed_camera"]                  = static_cast<bool> (true);         //is camera fixed relative to base?
    config["feature_detector_type"]         = std::string("SURF");              //SURF, SIFT, FAST, ... see misc.cpp
    config["feature_extractor_type"]        = std::string("SURF");              //SURF or SIFT
    config["matcher_type"]                  = std::string("FLANN");  //FLANN    //SIFTGPU or FLANN or BRUTEFORCE
    config["start_paused"]                  = static_cast<bool> (true);
    config["store_pointclouds"]             = static_cast<bool> (true);
    config["optimizer_iterations"]          = static_cast<int> (2);  //2
    config["subscriber_queue_size"]         = static_cast<int> (2);
    config["publisher_queue_size"]          = static_cast<int> (1);
    config["max_keypoints"]                 = static_cast<int> (1000);  //1000         //will also be used as max for SiftGPU
    config["min_keypoints"]                 = static_cast<int> (500);  //500
    config["min_matches"]                   = static_cast<int> (90);      //25     //if using SiftGPU and GLSL you should use max. 60 matches
    config["fast_max_iterations"]           = static_cast<int> (10);
    config["surf_max_iterations"]           = static_cast<int> (5);
    config["min_translation_meter"]         = static_cast<double> (0.05);	//0.05
    config["min_rotation_degree"]           = static_cast<int> (2.5);			//2.5
    config["min_time_reported"]             = static_cast<double> (1e9);        //by default, nothing should be reported
    config["squared_meshing_threshold"]     = static_cast<double> (0.0009);
    config["use_glwidget"]                  = static_cast<bool> (true);
    config["preserve_raster_on_save"]       = static_cast<bool> (false);
    config["connectivity"]                  = static_cast<int> (1e6);
    config["max_connections"]               = static_cast<int> (200);
    config["max_dist_for_inliers"]          = static_cast<double> (0.03);
    config["drop_async_frames"]             = static_cast<bool> (true); //false
    config["ransac_iterations"]             = static_cast<int> (1000); //1000
    config["use_gui"]                       = static_cast<bool> (false);
    config["use_wide"]                      = static_cast<bool> (true);
    config["concurrent_node_construction"]  = static_cast<bool> (true);
    config["concurrent_edge_construction"]  = static_cast<bool> (false);		// turn this off to broadcast nodes.  concurrent broadcasting of nodes not supported
    config["voxelfilter_size"]              = static_cast<double> (-0.1); //in meters. Values <= 0 deactivate
    config["wide_queue_size"]               = static_cast<int> (2);
    config["visualization_skip_step"]       = static_cast<int> (1);
    config["optimizer_skip_step"]           = static_cast<int> (1);   //1e6
    config["data_skip_step"]                = static_cast<int> (1);
    config["show_axis"]                     = static_cast<bool> (true); //use flann insteald of the brute force matcher
    config["depth_scaling_factor"]          = static_cast<double> (1.0); //Some kinects have a wrongly scaled depth
    config["keep_all_nodes"]                = static_cast<bool> (false); //Keep nodes with const motion assumption if too few inliers
    config["visualize_mono_depth_overlay"]  = static_cast<bool> (false); //Show Depth and Monochrome image as overlay in featureflow
    // ROSS PARAM
    config["pointcloud_skip_step"]          = static_cast<int> (5);
    config["neighbour_search_distance"]		= static_cast<double> (0.4);
    config["neighbour_search_angle"]		= static_cast<double> (40.0);
}

void ParameterServer::getValues() {
    map<string, boost::any>::const_iterator itr;
    for (itr = config.begin(); itr != config.end(); ++itr) {
        string name = itr->first;
        if (itr->second.type() == typeid(string)) {
            config[name] = getFromParameterServer<string> (pre + name,
                    boost::any_cast<string>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<string>(itr->second));
        } else if (itr->second.type() == typeid(int)) {
            config[name] = getFromParameterServer<int> (pre + name,
                    boost::any_cast<int>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<int>(itr->second));
        } else if (itr->second.type() == typeid(double)) {
            config[name] = getFromParameterServer<double> (pre + name,
                    boost::any_cast<double>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<double>(itr->second));
        } else if (itr->second.type() == typeid(bool)) {
            config[name] = getFromParameterServer<bool> (pre + name,
                    boost::any_cast<bool>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<bool>(itr->second));
        }
    }
}

void ParameterServer::checkValues() {
    if (get<string>("matcher_type").compare("SIFTGPU") == 0
            && get<bool>("concurrent_node_construction") == true) {
        config["concurrent_node_construction"] = static_cast<bool>(false);
        ROS_ERROR("Cannot use concurrent node construction with SiftGPU matcher! 'concurrent_node_construction' was set to false");
    }

    if (get<string>("matcher_type").compare("SIFTGPU") == 0
            && get<bool>("concurrent_edge_construction") == true) {
        config["concurrent_edge_construction"] = static_cast<bool>(false);
        ROS_ERROR("Cannot use concurrent edge construction with SiftGPU matcher! 'concurrent_edge_construction' was set to false");
    }
}
