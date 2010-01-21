cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(teleop_wii src/teleop_wii.cc)
target_link_libraries(${PROJECT_NAME} cwiid)
target_link_libraries(${PROJECT_NAME} pthread)
