cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(swissranger_driver)

rospack_add_library(swissranger src/swissranger.cpp)
target_link_libraries(swissranger usbSR)
