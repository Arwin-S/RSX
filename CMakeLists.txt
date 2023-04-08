cmake_minimum_required(VERSION 3.5 FATAL_ERROR)


project(segment_pcl)


find_package(PCL 1.2 REQUIRED)


# Variables set by the pcl config.cmake.in file
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (segment_pcl csv_io.cpp custom_ransac.cpp pcl_traversibility_score.cpp segment_pcl.cpp main.cpp)
target_link_libraries (segment_pcl ${PCL_LIBRARIES})