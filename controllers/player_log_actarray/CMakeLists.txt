cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
rosbuild_add_boost_directories()
rosbuild_genmsg()

rosbuild_add_executable (bin/player_actarray_log_to_msg src/player_actarray_log_to_msg.cpp)
rosbuild_add_executable (bin/actarray_cloud_assembler src/actarray_cloud_assembler.cpp)
#uncomment this line if you are from CS Department IX, TU Munich
rosbuild_add_executable (bin/sweep_laser_scan_cli_client src/sweep_laser_scan_cli_client.cpp)