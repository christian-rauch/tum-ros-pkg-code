cmake_minimum_required(VERSION 2.6)
include(rosbuild)
set(ROS_BUILD_TYPE Release)
rospack(flir_driver)

include_directories (/usr/local/include/unicap/)
link_directories (/usr/local/lib/)

rospack_add_library(flir src/flir.cpp)
target_link_libraries(flir unicap)
