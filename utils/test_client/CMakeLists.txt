PROJECT(testclient)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)


rosbuild_genmsg()
rosbuild_gensrv()

include_directories(source)

rosbuild_add_executable(test_cop source/test_cop.cpp)
rosbuild_add_executable(set_jlo source/set_jlo.cpp)
rosbuild_add_executable(query_jlo source/query_jlo.cpp)
