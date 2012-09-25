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
    config["feature_extractor"]        = std::string("SURF");            //SIFT, SURF or SIFTGPU
    config["descriptor_matcher"]       = std::string("dsf");      //Bruteforce, FLANN or SIFTGPU
	  config["max_keypoints"]            = static_cast<int> (1000);
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
