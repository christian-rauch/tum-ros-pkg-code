cmake_minimum_required(VERSION 2.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init ()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories (/usr/local/include/unicap/)
link_directories (/usr/local/lib/)

rosbuild_add_executable(composite_node src/composite_node.cpp)
rosbuild_add_executable(pcd_generator src/pcd_generator.cpp)

rosbuild_add_executable(snapshot src/snapshot.cpp)
target_link_libraries(snapshot)
