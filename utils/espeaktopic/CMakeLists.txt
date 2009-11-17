PROJECT(espeaktopic)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(easpeaktopic)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)


genmsg()
gensrv()

include_directories(source)


rospack_add_executable(espeaktopic source/espeak.cpp)
