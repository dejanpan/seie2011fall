cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug) #RelWithDebInfo)

rosbuild_init()


#set (CMAKE_C_COMPILER /usr/bin/gcc-4.4)
#set (CMAKE_CXX_COMPILER /usr/bin/g++-4.4)

#set (CMAKE_C_COMPILER /usr/local/bin/gcc)
#set (CMAKE_CXX_COMPILER /usr/local/bin/g++)

# next lines: copied from kidnapped_robot/CMakeLists.txt
find_package(PkgConfig)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)
include_directories(${SQLITE3_INCLUDE_DIRS})
link_directories(${SQLITE3_LIBRARY_DIRS})

# end of copy from kidnapped_robot


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(/home/engelhar/ros/kidnapped_robot/include)
link_directories(/home/engelhar/ros/external/projects-g2o/lib)

include_directories(/usr/include/suitesparse /home/engelhar/ros/external/projects-g2o  /home/engelhar/ros/external/projects-g2o/build )

# fuer slam3d interface:
# include_directories(/home/engelhar/ros/external/projects-radhar/radhar_alufr/slam3d/src)

# rosbuild_add_openmp_flags(tree_demo)
add_definitions(-Wall -Wno-missing-field-initializers -fopenmp)


#rosbuild_add_executable(global src/interface_test.cpp src/optimizer.cpp  src/ground_truth.cpp src/visualization.cpp src/cluster_manager.cpp src/FrameGenerator.cpp /home/engelhar/ros/kidnapped_robot/src/place_database.cpp)
# rosbuild_add_library(global_pose  src/optimizer.cpp  src/ground_truth.cpp src/visualization.cpp src/cluster_manager.cpp src/FrameGenerator.cpp /home/engelhar/ros/kidnapped_robot/src/place_database.cpp)

rosbuild_add_executable(global_slam src/main.cpp src/optimizer.cpp  src/ground_truth.cpp src/visualization.cpp src/cluster_manager.cpp src/FrameGenerator.cpp /home/engelhar/ros/kidnapped_robot/src/place_database.cpp)

target_link_libraries(global_slam ${SQLITE3_LIBRARIES}  g2o_core g2o_types_slam3d g2o_solver_csparse cxsparse)

