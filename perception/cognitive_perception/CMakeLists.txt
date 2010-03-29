PROJECT(cop)

cmake_minimum_required(VERSION 2.6)
INCLUDE("${CMAKE_MODULE_PATH}/MakeDocu.cmake" OPTIONAL)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()


find_package(PkgConfig)

pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)
include_directories(${LIBXML2_INCLUDE_DIR})
link_directories( ${LIBXML2_LIBRARY_DIRS} )
add_definitions( ${LIBXML2_CFLAGS} )

LINK_LIBRARIES(m lo cop)


rosbuild_add_executable(cop_srv ./source/main.cpp)

ROSBUILD_ADD_LIBRARY(cop
./source/AlgorithmEval.cpp      
./source/RelPose.cpp		 
./source/AttentionManager.cpp   
./source/AttentionAlgorithm.cpp   
./source/Signature.cpp
./source/Class.cpp
./source/SignatureDB.cpp
./source/RelPoseFactory.cpp
./source/Descriptor.cpp         
./source/Statistics.cpp
./source/Elem.cpp               
./source/TrackAlgorithm.cpp
./source/VisFinder.cpp
./source/ImageInputSystem.cpp   
./source/VisLearner.cpp
./source/XMLTag.cpp
./source/LocateAlgorithm.cpp  
./source/Object.cpp       
./source/RefineAlgorithm.cpp	
./source/ProveAlgorithm.cpp
./source/LogFile.cpp
./source/ROSjloComm.cpp
./source/ROSComm.cpp             
./source/File.cpp     
./source/Reading.cpp
./source/Sensor.cpp
./source/PerceptionPrimitive.cpp
)

set(DEBUG TRUE CACHE BOOL TRUE)

if (DEBUG )
  ADD_DEFINITIONS(-g -D_DEBUG -DBOOST_THREAD -DLOGFILE)
else (DEBUG)
  ADD_DEFINITIONS(-o3 -DBOOST_THREAD -DLOGFILE -DBOOST_1_35)
endif (DEBUG)

