cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
rosbuild_add_boost_directories()
rosbuild_genmsg()

rosbuild_add_executable (bin/player_actarray_log_to_msg src/player_actarray_log_to_msg.cpp)
rosbuild_add_executable (bin/actarray_cloud_assembler src/actarray_cloud_assembler.cpp)
