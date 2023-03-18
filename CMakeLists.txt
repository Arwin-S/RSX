cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(traversability_score)

find_package(PCL 1.2 REQUIRED)

# Variables set by the pcl config.cmake.in file
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (traversability_score main.cpp helpers.cpp)
target_link_libraries (traversability_score ${PCL_LIBRARIES})