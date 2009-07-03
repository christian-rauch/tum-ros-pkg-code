cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(rotational_reconstruction)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(rotational_reconstruction_node src/rotational_reconstruction_node.cpp src/sac_model_rotational.cpp)
rospack_add_executable(rotational_reconstructions_query_service src/query_rotational_reconstruction_service.cpp)

