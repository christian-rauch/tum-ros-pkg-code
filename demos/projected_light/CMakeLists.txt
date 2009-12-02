cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_add_executable(project_light src/project_light.cpp)
rosbuild_add_executable(disparity_view src/disparity_view.cpp)