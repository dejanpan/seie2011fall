/*
 * parameter_server.cpp
 *
 */
#include "frame_alignment/parameter_server.h"

using namespace std;

ParameterServer* ParameterServer::_instance = NULL;

ParameterServer::ParameterServer() {
	pre = "/frame_alignment/config/";

    defaultConfig();
    getValues();
}

ParameterServer* ParameterServer::instance() {
    if (_instance == NULL) {
        _instance = new ParameterServer();
    }

    return _instance;
}
void ParameterServer::defaultConfig() {
    config["feature_extractor"]               = std::string("SURF");            //SIFT, SURF
    config["feature_descriptor"]              = std::string("SURF");            //SIFT, SURF
    config["descriptor_matcher"]              = std::string("FLANN");      //Bruteforce, FLANN
	  config["max_keypoints"]                   = static_cast<int> (1000);
	  config["minimum_inliers"]                 = static_cast<int> (50);
    config["max_dist_for_inliers"]            = static_cast<double> (0.03);
    config["ransac_iterations"]               = static_cast<int> (1000); //1000
    config["save_features_image"]             = static_cast<bool> (false); //saves an image of the keypoints from the image to disk
    config["show_feature_matching"]           = static_cast<bool> (false); // displays a window showing matches before and after outlier removal
    config["use_openmp_normal_calculation"]   = static_cast<bool> (true);  // utilizes all cores for calculating normals.  I had problems once with this once so here it can be turned off
    config["use_ransac_to_initialize_icp"]    = static_cast<bool> (false);  // setting this to true uses the transformation from ransac as the starting point for the icp


    // --------------ICP settings------------------
    // weight between icp and visual features.  1.0 = features, 0.0 = ICP
    config["alpha"]                           = static_cast<double> (0.5);
    // the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    config["max_correspondence_dist"]         = static_cast<double> (0.05);
    // The maximum number of iterations (criterion 1)
    config["max_iterations"]                  = static_cast<int> (75);
    // The transformation epsilon (criterion 2)
    config["transformation_epsilon"]          = static_cast<double> (1e-8);
    // The euclidean distance difference epsilon (criterion 3)
    config["euclidean_fitness_epsilon"]       = static_cast<double> (0);
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
