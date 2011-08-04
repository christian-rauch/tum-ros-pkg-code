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


rosbuild_add_library(ias_drawer_executive_library src/RobotDriver.cpp src/Gripper.cpp src/Pressure.cpp src/RobotArm.cpp src/Poses.cpp src/AverageTF.cpp src/Torso.cpp src/OperateHandleController.cpp src/Head.cpp src/Approach.cpp src/OperateHandleServer.cpp src/DemoScripts.cpp src/Perception3d.cpp)

rosbuild_add_executable(ias_drawer_executive src/ias_drawer_executive.cpp)
target_link_libraries(ias_drawer_executive ias_drawer_executive_library)

#rosbuild_add_executable(marker src/Marker.cpp)
#rosbuild_add_executable(driver src/driver.cpp src/RobotDriver.cpp src/RobotArm.cpp  src/Pressure.cpp  src/Poses.cpp)

rosbuild_add_executable(operate_handle_server src/OperateHandleServer.cpp)
target_link_libraries(operate_handle_server ias_drawer_executive_library)


rosbuild_add_executable(test_client src/TestClient.cpp)

rosbuild_add_executable(segment_furniture_interactive src/SegmentFurnitureInteractive.cpp)
target_link_libraries(segment_furniture_interactive ias_drawer_executive_library)
# src/Perception3d.cpp src/RobotDriver.cpp src/Gripper.cpp src/Pressure.cpp src/RobotArm.cpp src/Poses.cpp src/AverageTF.cpp src/Torso.cpp src/OperateHandleController.cpp src/Head.cpp src/Approach.cpp)

rosbuild_add_executable(teleop_pr2_keyboard src/teleop_pr2_keyboard.cpp) 
target_link_libraries(teleop_pr2_keyboard ias_drawer_executive_library)
# src/RobotDriver.cpp src/Gripper.cpp src/Pressure.cpp src/RobotArm.cpp src/Poses.cpp src/AverageTF.cpp src/Torso.cpp src/OperateHandleController.cpp src/Head.cpp src/Approach.cpp)

#rosbuild_add_executable(grasp_cluster_client src/GraspClusterClient.cpp)
