cmake_minimum_required(VERSION 2.4.6)
##########################################################
#comment player_bender_dummy if not on bender(roboradig31)
##########################################################
execute_process(COMMAND hostname
                OUTPUT_VARIABLE ROS_HOSTNAME)
message( "Hostname: ${ROS_HOSTNAME}" )
if("${ROS_HOSTNAME}" MATCHES "roboradig31")
message( "OK, I am on ${ROS_HOSTNAME}" )
#rosbuild_add_executable (bin/sweep_laser_scan_cli_client src/sweep_laser_scan_cli_client.cpp)
else("${ROS_HOSTNAME}" MATCHES "roboradig31")
message( "Not on roboradig31" )
execute_process(COMMAND python $ENV{PWD}/../src/scripts/search_replace.py $ENV{PWD}/../manifest.xml  $ENV{PWD}/../tmp 0)
endif("${ROS_HOSTNAME}" MATCHES "roboradig31")

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
rosbuild_link_boost(bin/actarray_cloud_assembler filesystem)
rosbuild_add_executable (bin/multi_pcd_to_msg src/multi_pcd_to_msg.cpp)
rosbuild_link_boost(bin/multi_pcd_to_msg filesystem)
rosbuild_add_executable (bin/pointcloud_delimiter src/pointcloud_delimiter.cpp)
rosbuild_link_boost(bin/pointcloud_delimiter filesystem)
##########################################################
#Compiles only on blue B21 robot from CS Department IX, TU Munich
##########################################################
execute_process(COMMAND hostname
                OUTPUT_VARIABLE ROS_HOSTNAME)

if("${ROS_HOSTNAME}" MATCHES "roboradig31")
rosbuild_add_executable (bin/sweep_laser_scan_cli_client src/sweep_laser_scan_cli_client.cpp)
#undo comment from above
else("${ROS_HOSTNAME}" MATCHES "roboradig31")
execute_process(COMMAND python $ENV{PWD}/../src/scripts/search_replace.py $ENV{PWD}/../manifest.xml  $ENV{PWD}/../tmp 1)
endif("${ROS_HOSTNAME}" MATCHES "roboradig31")
