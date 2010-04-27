cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug) #RelWithDebInfo)
rosbuild_init()

rosbuild_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_executable (wall_filter_node src/wall_filter.cpp)
rosbuild_add_executable (table_object_detector_passive_node src/table_object_detector_passive.cpp)
rosbuild_add_executable (box_filter_node src/box_filter.cpp)
rosbuild_add_executable (msg_to_pcd_node src/msg_to_pcd.cpp)
rosbuild_add_executable (msg_to_pcd_joy_node src/msg_to_pcd_joy.cpp)
rosbuild_add_executable (table_msg_split_node src/table_msg_split.cpp)
rosbuild_add_executable (get_line_id_hokuyo_node src/get_line_id_hokuyo.cpp)
rosbuild_add_executable (triangular_mesh_to_vtk_node src/triangular_mesh_to_vtk.cpp)
