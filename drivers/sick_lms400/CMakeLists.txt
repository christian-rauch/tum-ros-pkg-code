cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init ()
rosbuild_gensrv()

rosbuild_add_library (sick_lms400 src/sick_lms400.cpp)

rosbuild_add_executable (bin/lms400_node src/lms400_node.cpp)
target_link_libraries (bin/lms400_node sick_lms400)

