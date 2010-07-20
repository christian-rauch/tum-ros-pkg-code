cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

file(GLOB def_files ${CMAKE_CURRENT_SOURCE_DIR}/defs/*.xml
                    ${CMAKE_CURRENT_SOURCE_DIR}/robots/*groups*.xml)

file(GLOB xacros ${CMAKE_CURRENT_SOURCE_DIR}/robots/*.urdf.xml)

rosbuild_find_ros_package(xacro)

set(outfiles "")

foreach( _xacro ${xacros} )
  string(REPLACE ".urdf." ".expanded." _outfile ${_xacro})
  add_custom_command(
    OUTPUT ${_outfile}
    COMMAND ${xacro_PACKAGE_PATH}/xacro.py
    ARGS ${_xacro} > ${_outfile}
    DEPENDS ${_xacro} ${def_files})
  list(APPEND outfiles ${_outfile})
endforeach( _xacro)

add_custom_target( expanded_defs ALL DEPENDS ${outfiles} )
