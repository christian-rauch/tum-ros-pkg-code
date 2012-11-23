cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_library(fri src/FRIComm.cc src/YARPComm.cc src/FRICheck.cc src/ROSComm.cc src/CartesianImpedance.cc)

rosbuild_add_executable(fri src/main.cc)
target_link_libraries(fri pthread)
