cmake_minimum_required(VERSION 3.1.0)

project(viewer)

find_package(PCL 1.9 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (real_sense_viewer real_sense_viewer.cpp)
target_link_libraries (real_sense_viewer ${PCL_LIBRARIES})
