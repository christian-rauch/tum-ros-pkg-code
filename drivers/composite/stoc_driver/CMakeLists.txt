cmake_minimum_required(VERSION 2.6)
include(rosbuild)
set(ROS_BUILD_TYPE Release)
rospack(stoc_driver)

rospack_add_library(stoc src/stoc.cpp)
target_link_libraries(stoc svscap svs svscalc)
