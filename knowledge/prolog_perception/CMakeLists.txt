cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Must have <SWI-Prolog.h>
INCLUDE(CheckIncludeFiles)
CHECK_INCLUDE_FILES ("/usr/lib/swi-prolog/include/SWI-Prolog.h" HAVE_SWI_PROLOG_H)
if(HAVE_SWI_PROLOG_H)
  message(STATUS "<SWI-Prolog.h> found")
  INCLUDE_DIRECTORIES("/usr/lib/swi-prolog/include/")
else(HAVE_SWI_PROLOG_H)
  message(FATAL_ERROR "Can't proceed without <SWI-Prolog.h>")
endif(HAVE_SWI_PROLOG_H)

#rosbuild_add_executable(query_lo_cop src/cpp/query_lo_cop.cpp)
#rosbuild_add_executable(query_table_memory_write_to_file src/cpp/query_table_memory_write_to_file.cpp)
#rosbuild_add_library(ros_query_tabletop_missing_objects src/cpp/query_tabletop_missing_objects.cpp)
#rosbuild_add_library(ros_query_plane src/cpp/query_plane.cpp)
#rosbuild_add_library(ros_query_table_objects src/cpp/query_table_objects.cpp)
#rosbuild_add_library(ros_query_cluster_color src/cpp/query_cluster_color.cpp)
rosbuild_add_library(ros_query_table_memory src/cpp/query_table_memory.cpp)


###########################################################
#
# code for replacings paths in the prolog module
#


# replace paths for includes in prolog files
rosbuild_find_ros_package(prolog_perception)
set( PL_PATH_PREFIX "${prolog_perception_PACKAGE_PATH}/" )

file(GLOB inputs prolog/*.in)

foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )

file(GLOB inputs owl/*.in)

foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )