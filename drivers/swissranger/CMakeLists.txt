cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(swissranger)

rospack_add_library(swissranger src/swissranger.cpp)
target_link_libraries(swissranger mesasr)

rospack_add_executable (bin/swissranger_test_node src/swissranger_test_node.cpp)
target_link_libraries (bin/swissranger_test_node swissranger)

rospack_add_executable (bin/swissranger_viewer src/swissranger_viewer.cpp)
target_link_libraries (bin/swissranger_viewer swissranger)
