cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(information_gain)
set(PCL_DIR "usr/local/share/pcl-1.5/PCLConfig.cmake")
find_package(PCL 1.3 REQUIRED COMPONENTS common io filters registration)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(information_gain src/information_gain.cpp)

target_link_libraries(information_gain 
											pcl_sample_consensus 
											pcl_segmentation 
											pcl_surface 
											pcl_visualization
											${PCL_LIBRARIES} 
											${PCL_COMMON_LIBRARIES} 
											${PCL_IO_LIBRARIES} 
											${PCL_FILTERS_LIBRARIES} 
											${PCL_REGISTRATION_LIBRARIES} 
											${PCL_SEGMENTATION_LIBRARIES})

#target_link_libraries(information_gain pcl_sample_consensus pcl_segmentation pcl_surface  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})
