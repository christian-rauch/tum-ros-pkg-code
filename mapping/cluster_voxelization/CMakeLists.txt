cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(cluster_voxelization)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(cluster_voxelization_node src/cluster_voxelization.cpp)
rospack_add_executable(clusters_query_service src/query_cluster_voxelization_service.cpp)

