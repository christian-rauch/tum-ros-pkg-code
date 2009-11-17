PROJECT(jlo)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(jlo)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

#Add Located Object 
#pkg_check_modules(LO REQUIRED liblo)
#
#MESSAGE(STATUS "lo includes:  ${LO_INCLUDE_DIRS}")
#include_directories(${LO_INCLUDE_DIRS})
#
#MESSAGE(STATUS "lo cflags:  ${LO_CFLAGS}")
#add_definitions( ${LO_CFLAGS})
#
#MESSAGE(STATUS "lo ld: ${LO_LIBRARY_DIRS}")
#link_directories(${LO_LIBRARY_DIRS})

find_package(PkgConfig)

pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)
include_directories(${LIBXML2_INCLUDE_DIR})
link_directories( ${LIBXML2_LIBRARY_DIRS} )
add_definitions( ${LIBXML2_CFLAGS} )



genmsg()
gensrv() 

include_directories(source)

rospack_add_executable(jlo source/main.cpp source/RosLoService.cpp)

target_link_libraries(jlo ${LO_LIBRARIES} ${LIBXML2_LDFLAGS})
