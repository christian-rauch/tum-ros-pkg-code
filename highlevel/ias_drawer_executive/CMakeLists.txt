cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(ias_drawer_executive src/ias_drawer_executive.cpp)
rosbuild_add_executable(ias_drawer_executive src/RobotDriver.cpp)
rosbuild_add_executable(ias_drawer_executive src/Gripper.cpp)
rosbuild_add_executable(ias_drawer_executive src/Pressure.cpp)
rosbuild_add_executable(ias_drawer_executive src/RobotArm.cpp)
rosbuild_add_executable(ias_drawer_executive src/Poses.cpp)
rosbuild_add_executable(ias_drawer_executive src/AverageTF.cpp)
rosbuild_add_executable(ias_drawer_executive src/Torso.cpp)
rosbuild_add_executable(ias_drawer_executive src/OperateHandleController.cpp)
rosbuild_add_executable(ias_drawer_executive src/Head.cpp)
rosbuild_add_executable(ias_drawer_executive src/Approach.cpp)
rosbuild_add_executable(ias_drawer_executive src/DemoScripts.cpp)

rosbuild_add_executable(constraints src/Constraints.cpp)

rosbuild_add_executable(operate_handle_server src/OperateHandleServer.cpp)
rosbuild_add_executable(operate_handle_server src/RobotDriver.cpp)
rosbuild_add_executable(operate_handle_server src/Gripper.cpp)
rosbuild_add_executable(operate_handle_server src/Pressure.cpp)
rosbuild_add_executable(operate_handle_server src/RobotArm.cpp)
rosbuild_add_executable(operate_handle_server src/Poses.cpp)
rosbuild_add_executable(operate_handle_server src/AverageTF.cpp)
rosbuild_add_executable(operate_handle_server src/Torso.cpp)
rosbuild_add_executable(operate_handle_server src/OperateHandleController.cpp)
rosbuild_add_executable(operate_handle_server src/Head.cpp)
rosbuild_add_executable(operate_handle_server src/Approach.cpp)
rosbuild_add_executable(operate_handle_server src/DemoScripts.cpp)
rosbuild_add_executable(test_client src/TestClient.cpp)
rosbuild_add_executable(SnapMapICP src/SnapMapICP.cpp)

#rosbuild_add_library(ias_drawer_executive_library src/RobotDriver.cpp src/Gripper.cpp src/Pressure.cpp src/RobotArm.cpp src/Poses.cpp src/AverageTF.cpp src/Torso.cpp src/OperateHandleController.cpp src/Head.cpp src/Approach.cpp src/OperateHandleServer.cpp src/DemoScripts.cpp)

#target_link_libraries(ias_drawer_executive curses)
