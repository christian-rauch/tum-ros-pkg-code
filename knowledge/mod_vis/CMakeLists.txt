cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

add_classpath(${PROJECT_SOURCE_DIR}/lib/controlP5.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/core.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/vecmath.jar)

rosbuild_find_ros_package(jpl)
add_classpath(${jpl_PACKAGE_PATH}/jpl/jpl.jar)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)