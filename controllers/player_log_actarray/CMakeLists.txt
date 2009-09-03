cmake_minimum_required(VERSION 2.4.6)
##########################################################
#comment player_bender_dummy if not on bender(roboradig31)
##########################################################
execute_process(COMMAND hostname
                OUTPUT_VARIABLE ROS_HOSTNAME)
message( "Hostname: ${ROS_HOSTNAME}" )
if("${ROS_HOSTNAME}" STREQUAL "roboradig31")
#rosbuild_add_executable (bin/sweep_laser_scan_cli_client src/sweep_laser_scan_cli_client.cpp)
else("${ROS_HOSTNAME}" STREQUAL "roboradig31")
execute_process(COMMAND python $ENV{PWD}/../src/scripts/search_replace.py $ENV{PWD}/../manifest.xml  $ENV{PWD}/../tmp 0)
endif("${ROS_HOSTNAME}" STREQUAL "roboradig31")

##########################################################
#ROS stuff
##########################################################
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
rosbuild_add_boost_directories()
rosbuild_genmsg()

rosbuild_add_executable (bin/player_actarray_log_to_msg src/player_actarray_log_to_msg.cpp)
rosbuild_add_executable (bin/actarray_cloud_assembler src/actarray_cloud_assembler.cpp)

##########################################################
#Compiles only on blue B21 robot from CS Department IX, TU Munich
##########################################################
execute_process(COMMAND hostname
                OUTPUT_VARIABLE ROS_HOSTNAME)

if("${ROS_HOSTNAME}" STREQUAL "roboradig31")
rosbuild_add_executable (bin/sweep_laser_scan_cli_client src/sweep_laser_scan_cli_client.cpp)
#undo comment from above
else("${ROS_HOSTNAME}" STREQUAL "roboradig31")
execute_process(COMMAND python $ENV{PWD}/../src/scripts/search_replace.py $ENV{PWD}/../manifest.xml  $ENV{PWD}/../tmp 1)
endif("${ROS_HOSTNAME}" STREQUAL "roboradig31")
