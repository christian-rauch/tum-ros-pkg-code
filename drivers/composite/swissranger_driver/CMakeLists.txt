cmake_minimum_required(VERSION 2.6)
include(rosbuild)
set(ROS_BUILD_TYPE Release)
rospack(swissranger_driver)

rospack_add_library(swissranger src/swissranger.cpp)
target_link_libraries(swissranger usbSR)
