cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set (ROS_BUILD_TYPE Debug)
rosbuild_init ()
rosbuild_add_boost_directories ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_gensrv ()
rosbuild_add_executable (table_pose src/table_pose.cpp)

