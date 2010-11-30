cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_gensrv()

# depend on visualization packages from mod_vis
rosbuild_find_ros_package(mod_vis)
add_classpath(${mod_vis_PACKAGE_PATH}/bin)
add_classpath(${mod_vis_PACKAGE_PATH}/lib/core.jar)
add_classpath(${mod_vis_PACKAGE_PATH}/lib/controlP5.jar)
add_classpath(${mod_vis_PACKAGE_PATH}/lib/vecmath.jar)

add_runtime_classpath(${mod_vis_PACKAGE_PATH}/bin)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/core.jar)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/controlP5.jar)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/vecmath.jar)


add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

# create scripts
rospack_add_java_executable(meshlabel_node edu.tum.cs.ias.knowrob.ObjectLabelingCanvas)
rospack_add_java_executable(meshlabel_test_client edu.tum.cs.ias.knowrob.MeshLabelTestClient)
