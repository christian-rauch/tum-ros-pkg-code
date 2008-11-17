cmake_minimum_required(VERSION 2.6)
include(rosbuild)
set(ROS_BUILD_TYPE Debug)
rospack(visualizer_fusion)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Move this as a dependency for this ROS node
find_package(VTK REQUIRED)
if(NOT VTK_USE_RENDERING)
  message(FATAL_ERROR "Example ${PROJECT_NAME} requires VTK_USE_RENDERING.")
endif(NOT VTK_USE_RENDERING)
include(${VTK_USE_FILE})

set(VTK_LIBS vtkCommon vtkRendering vtkWidgets vtkHybrid vtkGraphics)

rospack_add_executable(visualizer_fusion src/visualizer_fusion.cpp src/vtk_tools.cpp)
target_link_libraries(visualizer_fusion ${VTK_LIBS})

rospack_add_executable(param_visualizer src/param_visualizer.cpp src/terminal.cpp)
target_link_libraries(param_visualizer)
