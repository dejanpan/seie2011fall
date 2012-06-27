#!/bin/sh

roslaunch ias_classifier_manager manager.launch
rosrun object_part_decomposition init_classifier.sh
rosrun object_part_decomposition classify_scene_service
python ../src/find_corners.py
