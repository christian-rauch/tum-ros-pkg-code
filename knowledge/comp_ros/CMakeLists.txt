cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


rosbuild_init()

add_classpath(${comp_ros_PACKAGE_PATH}/lib/ros_client.jar)

rosbuild_find_ros_package(inf9_knowledge_base)
set( OWL_PATH_PREFIX ${inf9_knowledge_base_PACKAGE_PATH} )

file(GLOB inputs owl/*.in)

foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )
