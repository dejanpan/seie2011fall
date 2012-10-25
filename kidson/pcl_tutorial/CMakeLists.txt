cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include_directories(/usr/include/vtk-5.8/)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable (euclidean_cluster_test src/euclidean_cluster_test.cpp)
#rosbuild_add_executable (kdtree_search src/kdtree_search.cpp)
#rosbuild_add_executable (passthrough_rand src/passthrough_rand.cpp)
#rosbuild_add_executable (passthrough_RGBD src/passthrough_RGBD.cpp)
#rosbuild_add_executable (voxFilt_RGBD src/voxFilt_RGBD.cpp)
rosbuild_add_executable (voxFilt_PCD src/voxFilt_PCD.cpp)
rosbuild_add_executable (import_mesh src/import_mesh.cpp)
target_link_libraries(import_mesh libboost_system.so)
#rosbuild_add_executable (outliers_PCD src/outliers_PCD.cpp)
#rosbuild_add_executable (project_inliers_rand src/project_inliers_rand.cpp)
#rosbuild_add_executable (extract_indices_PCD src/extract_indices_PCD.cpp)
##rosbuild_add_executable (remove_outliers_rand src/remove_outliers_rand.cpp)
#rosbuild_add_executable (pcd_read src/pcd_read.cpp)
#rosbuild_add_executable (pcd_write src/pcd_write.cpp)
#rosbuild_add_executable (concatenate_clouds src/concatenate_clouds.cpp)
#rosbuild_add_executable (cloudTransformer src/cloudTransformer.cpp)
#rosbuild_add_executable (planar_segmentation src/planar_segmentation.cpp)
#rosbuild_add_executable (cylinder_segmentation src/cylinder_segmentation.cpp)
#rosbuild_add_executable (cluster_extraction src/cluster_extraction.cpp)
#rosbuild_add_executable (icp_pcl src/icp_pcl.cpp)
#rosbuild_add_executable (point_to_plane_test src/point_to_plane_test.cpp)
#rosbuild_add_executable (correspond_test src/correspond_test.cpp)
#rosbuild_add_executable (icp_test src/icp_test.cpp)
#rosbuild_add_executable (icp_testbed src/icp_testbed.cpp)
#rosbuild_add_executable (normal_estimation src/normal_estimation.cpp)
#rosbuild_add_executable (apply_transformation src/apply_transformation.cpp)
#rosbuild_add_executable (correspondence_grouping src/correspondence_grouping.cpp)
#target_link_libraries(correspondence_grouping libvtkCommon.so libvtkRendering.so libvtkFiltering.so libpcl_recognition.so)
#rosbuild_add_executable (extractPCDFromBag src/extractPCDFromBag.cpp)
#rosbuild_add_executable (handleExtraction src/handleExtraction.cpp)
#rosbuild_add_executable (icp_handleExtraction src/icp_handleExtraction.cpp)
#rosbuild_add_executable (normal_distributions_transform src/normal_distributions_transform.cpp)
#target_link_libraries(normal_distributions_transform libvtkCommon.so libvtkRendering.so libvtkFiltering.so libpcl_recognition.so)
#rosbuild_add_executable (readFromBag src/readFromBag.cpp)
