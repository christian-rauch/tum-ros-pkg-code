cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init ()

include_directories (/usr/local/include/unicap/)
link_directories (/usr/local/lib/)

rosbuild_add_library(flir src/flir.cpp)
target_link_libraries(flir unicap)
