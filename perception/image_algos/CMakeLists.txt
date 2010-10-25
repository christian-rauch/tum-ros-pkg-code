cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE RelWithDebInfo)
#rosbuild_init()
#gensrv()
rosbuild_init()
rosbuild_add_boost_directories()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ROSBUILD_ADD_LIBRARY (image_algos
  src/color_find_hsv.cpp
  src/pcd_to_image_projector_algo.cpp
  src/register_plugins.cpp
)


rosbuild_add_executable (find_color_hsv_node src/color_find_hsv.cpp)
set_target_properties (find_color_hsv_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

#TODO: make this one run as node
#rosbuild_add_executable (pcd_to_image_projector_node src/pcd_to_image_projector_algo.cpp)
#set_target_properties (pcd_to_image_projector_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)
rosbuild_add_executable (pointcloud_to_image_projector_halcon_node src/pointcloud_to_image_projector_halcon.cpp)
rosbuild_add_executable (pointcloud_to_image_projector_opencv_node src/pointcloud_to_image_projector_opencv.cpp)
rosbuild_add_executable (pointcloud_colorizer_node src/pointcloud_colorizer.cpp)
rosbuild_add_executable (pointcloud_colorizer_with_head_movement_node src/pointcloud_colorizer_with_head_movement.cpp)

rosbuild_add_executable (openCv_to_ros utils/openCV_to_ros.cpp)
#rosbuild_add_executable (bgfg_codebook_node src/bgfg_codebook_node.cpp)
#rosbuild_add_compile_flags(openCv_to_ros "-pg")
#rosbuild_add_link_flags(openCv_to_ros "-pg")



# rosbuild_add_executable(cmvision
#   src/cmvision.cc
#   src/conversions.c
#   src/cmvision_color_blob_finder.cpp
#   src/main.cpp)
# rosbuild_add_executable(blob src/blob_detection_hsv.cpp)

rosbuild_add_executable (image_algos_main src/image_algos_main.cpp)
target_link_libraries (image_algos_main image_algos)
