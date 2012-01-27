cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

add_custom_command(OUTPUT ${EXECUTABLE_OUTPUT_PATH}/MicroEdgeCorrectionTable.bin
	COMMAND $(MAKE) -C ${PROJECT_SOURCE_DIR}/EdgeDetection
	COMMAND ${PROJECT_SOURCE_DIR}/EdgeDetection/bin/CreateMicroEdgeCorrectionTable
	COMMAND cp MicroEdgeCorrectionTable.bin ${EXECUTABLE_OUTPUT_PATH}/MicroEdgeCorrectionTable.bin
)

add_custom_target(MicroEdgeCorrectionTable
	DEPENDS ${EXECUTABLE_OUTPUT_PATH}/MicroEdgeCorrectionTable.bin
)

rosbuild_add_executable(tracker
	src/PoseEstimator.cpp
	src/MarkerRefiner.cpp
	src/MarkerPublisher.cpp
	src/TransformPublisher.cpp
	src/tracker.cpp
	src/EdgeDetection/AdaptiveRasterizer.cpp
	src/EdgeDetection/EdgeImage.cpp
	src/EdgeDetection/Image.cpp
	src/EdgeDetection/MacroEdgeDetector.cpp
	src/EdgeDetection/MemoryImage.cpp
	src/EdgeDetection/MicroEdgeCorrectionTable.cpp
	src/EdgeDetection/MicroEdgeDetector.cpp
	src/EdgeDetection/MicroEdgeMap.cpp
	src/EdgeDetection/MomentMask.cpp
	src/EdgeDetection/OwnedImage.cpp
	src/aruco/board.cpp
	src/aruco/boarddetector.cpp
	src/aruco/cameraparameters.cpp
	src/aruco/cvdrawingutils.cpp
	src/aruco/marker.cpp
	src/aruco/markerdetector.cpp
	src/rpp/librpp.cpp
	src/rpp/rpp.cpp
	src/rpp/rpp_quintic.cpp
	src/rpp/rpp_svd.cpp
	src/rpp/rpp_vecmat.cpp
)


rosbuild_add_library(arucotracker
	src/PoseEstimator.cpp
	src/MarkerRefiner.cpp
	src/MarkerPublisher.cpp
	src/TransformPublisher.cpp
	src/EdgeDetection/AdaptiveRasterizer.cpp
	src/EdgeDetection/EdgeImage.cpp
	src/EdgeDetection/Image.cpp
	src/EdgeDetection/MacroEdgeDetector.cpp
	src/EdgeDetection/MemoryImage.cpp
	src/EdgeDetection/MicroEdgeCorrectionTable.cpp
	src/EdgeDetection/MicroEdgeDetector.cpp
	src/EdgeDetection/MicroEdgeMap.cpp
	src/EdgeDetection/MomentMask.cpp
	src/EdgeDetection/OwnedImage.cpp
	src/aruco/board.cpp
	src/aruco/boarddetector.cpp
	src/aruco/cameraparameters.cpp
	src/aruco/cvdrawingutils.cpp
	src/aruco/marker.cpp
	src/aruco/markerdetector.cpp
	src/rpp/librpp.cpp
	src/rpp/rpp.cpp
	src/rpp/rpp_quintic.cpp
	src/rpp/rpp_svd.cpp
	src/rpp/rpp_vecmat.cpp
)

add_dependencies(tracker MicroEdgeCorrectionTable)

rosbuild_add_executable(create_marker
    src/aruco/marker.cpp
    src/aruco/utils/aruco_create_marker.cpp
)

