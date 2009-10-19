cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug) #RelWithDebInfo)
rosbuild_init ()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_executable (table_memory src/table_memory.cpp)
rosbuild_add_executable (find_clusters src/find_clusters.cpp)

rosbuild_add_executable (msg_to_pcd src/msg_to_pcd.cpp)
#rosbuild_add_executable (msg_to_pcd_joy src/msg_to_pcd_joy.cpp)
rosbuild_add_executable (table_msg_split src/table_msg_split.cpp)
rosbuild_add_executable (object_reconstruction src/object_reconstruction.cpp src/Sherlock.cpp src/Shape.cpp)
#rosbuild_add_executable (msg_to_pcd_joy_registration src/msg_to_pcd_registration.cpp)
# src/icp.cc)

