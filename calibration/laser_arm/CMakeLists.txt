cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(laser_arm)
rospack_add_boost_directories()

rospack_add_executable (bin/laser_arm_calib_node src/laser_arm_calib.cpp)
