cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(player_log_actarray)
rospack_add_boost_directories()
genmsg()
rospack_add_executable (bin/player_actarray_log_to_msg src/actarray_log_to_msg.cpp)
