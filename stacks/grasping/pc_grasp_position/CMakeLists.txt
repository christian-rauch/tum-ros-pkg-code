PROJECT(trans_pc_jlo)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()


include_directories(source)
ADD_DEFINITIONS(-o3)

rosbuild_add_executable(grasp_pcd src/grasp_pcd.cpp)


