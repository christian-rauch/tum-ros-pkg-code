PROJECT(testclient)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(test_client)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)


genmsg()
gensrv()

include_directories(source)

rospack_add_executable(test_cop source/main.cpp)
rospack_add_executable(test_jlo source/main_jlo.cpp)
rospack_add_executable(query_jlo source/main_jlo2.cpp)
