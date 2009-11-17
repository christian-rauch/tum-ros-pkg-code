PROJECT(vision_msgs)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(vision_msgs)

genmsg()
gensrv() 

