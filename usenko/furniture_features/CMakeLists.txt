cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set (srcs
     src/conditional_removal.cpp
     src/sgf.cpp
     )

set (incs
     include pcl/${SUBSYS_NAME}/conditional_removal.h
     include pcl/${SUBSYS_NAME}/sgf1.h
     include pcl/${SUBSYS_NAME}/sgf2.h
     include pcl/${SUBSYS_NAME}/sgf3.h
     include pcl/${SUBSYS_NAME}/sgf4.h
     include pcl/${SUBSYS_NAME}/sgf5.h
     include pcl/${SUBSYS_NAME}/sgf6.h
     include pcl/${SUBSYS_NAME}/sgf7.h
     include pcl/${SUBSYS_NAME}/sgf8.h
     include pcl/${SUBSYS_NAME}/sgf9.h
    )

set (impl_incs
     include/pcl/${SUBSYS_NAME}/impl/conditional_removal.hpp
     #include/pcl/${SUBSYS_NAME}/impl/sgf.hpp
     )

rosbuild_add_executable(sgf_test src/sgf_test.cpp)
#rosbuild_add_executable(classify_nn src/classify_nn.cpp)
#rosbuild_add_executable(classify_svm src/classify_svm.cpp)

