cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include_directories("/usr/include/openni/")
#link_directories("/usr/lib/libvtkCommon.so.5.6")
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

#rosbuild_add_executable(main_multiple_tracking src/main_multiple_tracking.cpp src/3d_tracking_lines.cpp src/PrimitivesExtract.cpp)
rosbuild_add_executable(main_testing src/main_testing.cpp src/3d_tracking_lines.cpp src/PrimitivesExtract.cpp)
#target_link_libraries(main_multiple_tracking libvtkCommon.so libvtkFiltering.so libvtkRendering.so pcl_tracking)
target_link_libraries(main_testing libvtkCommon.so libvtkFiltering.so libvtkRendering.so pcl_tracking)
#target_link_libraries(example ${PROJECT_NAME})
#target_link_libraries(main_multiple_tracking ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} pcl_common pcl_io pcl_surface pcl_visualization pcl_filters pcl_features pcl_segmentation pcl_tracking pcl_search pcl_kdtree pcl_sample_consensus)
