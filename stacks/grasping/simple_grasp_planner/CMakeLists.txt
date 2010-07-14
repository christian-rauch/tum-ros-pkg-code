PROJECT("Simple Grasp Planner")

cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(spg )

INCLUDE_DIRECTORIES("include")

rosbuild_add_library(sgp SHARED src/SimpleGraspPlanner.c)


