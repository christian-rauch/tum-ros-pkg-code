cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(swissranger_visualizer)

rospack_add_executable (bin/swissranger_viewer src/swissranger_viewer.cpp)
