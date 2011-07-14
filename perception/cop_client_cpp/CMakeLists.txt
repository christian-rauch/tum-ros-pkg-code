PROJECT(testclient)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)


rosbuild_genmsg()
rosbuild_gensrv()

include_directories(include)

rosbuild_add_library(copclientcpp src/cop_client.cpp)

LINK_LIBRARIES(copclientcpp)
rosbuild_add_executable(test_cop src/test_cop.cpp)
rosbuild_add_executable(refinements src/refinements.cpp)
