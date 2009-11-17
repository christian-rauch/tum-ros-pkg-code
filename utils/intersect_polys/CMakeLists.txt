PROJECT(intersect_polygons)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(intersect_polygons)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)


genmsg()
gensrv()

include_directories(source)
include_directories($ENV{HALCONROOT}/include
$ENV{HALCONEXTENSIONS}/include)
link_directories($ENV{HALCONROOT}/lib/$ENV{HALCONARCH}
               $ENV{HALCONEXTENSIONS}/lib/$ENV{HALCONARCH})
link_libraries(halconcpp)



rospack_add_executable(intersect_polygons source/intersect_polygons.cpp)
