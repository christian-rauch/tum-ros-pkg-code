PROJECT(cop)

cmake_minimum_required(VERSION 2.6)

INCLUDE("${CMAKE_MODULE_PATH}/MakeDocu.cmake" OPTIONAL)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()

set (SWISSRANGER_AVAILABLE TRUE CACHE BOOL "Is a swissranger and plane_clusters running in your system?")
IF ($(SWISSRANGER_AVAILABLE))
	set (SWISS_DEFS "-DSWISS_RANGER_SERVICE")
ENDIF ($(SWISSRANGER_AVAILABLE))
#find_package(Boost)
#include_directories( ${Boost_INCLUDE_DIRS} )
#link_directories( ${Boost_LIBRARY_DIRS} )

find_package(PkgConfig)

pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)
include_directories(${LIBXML2_INCLUDE_DIR})
link_directories( ${LIBXML2_LIBRARY_DIRS} )
add_definitions( ${LIBXML2_CFLAGS} )

#HALCON include + libs
INCLUDE_DIRECTORIES($ENV{HALCONROOT}/include)
LINK_DIRECTORIES($ENV{HALCONROOT}/lib/$ENV{HALCONARCH})

#string(REPLACE : ; halconextension_lib $ENV{HALCONEXTENSIONS};kl)
#
#foreach(loop_var ${halconextension_list})
#	INCLUDE_DIRECTORIES(${loop_var}/include)
#	LINK_DIRECTORIES(${loop_var}/lib/$ENV{HALCONARCH})
#endforeach(loop_var)



INCLUDE_DIRECTORIES(
../3rdParty/stdint.h/
../3rdParty/tclap-1.1.0/include
./include
../include
../Grabber/PTU/include
../3rdParty/iconv-1.9.2.win32/include
../../3rdparty/dxflib/src
../3rdParty/dxflib/src
../stereomatching/descriptor3d/include
../3rdParty/deform_shape/include
../3rdParty/planar_pose_estimation/include
../easymatching/descriptor3d/include
/usr/stud/tavcar/local/etch-amd64/include/player-2.2/
${CMAKE_SOURCE_DIR}/util)
LINK_DIRECTORIES(
/usr/stud/tavcar/local/etch-amd64/lib
../Grabber/PTU
../../3rdParty/dxflib/lib
../../../3rdparty/dxflib/lib
../../stereomatching/descriptor3d/lib/$ENV{HALCONARCH}
../../lib)


#LINK_LIBRARIES(m halcon parhalconcpp lo cop  dxflib playerc)
LINK_LIBRARIES(m halcon halconcpp lo cop)

configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/libcop.pc.in ${CMAKE_CURRENT_SOURCE_DIR}/libcop.pc )
install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/libcop.pc DESTINATION lib/pkgconfig )


rosbuild_add_executable(cop_srv ./source/main.cpp)
#ADD_EXECUTABLE(cop_write ./testprgs/main.cpp)
ROSBUILD_ADD_LIBRARY(cop
./source/Action.cpp             ./source/Particle.cpp
./source/AlgorithmEval.cpp       ./source/BlobLocalizer.cpp
./source/RelPose.cpp		    ./source/RegionOI.cpp
 ./source/DxfReader.cpp          ./source/Blob.cpp
./source/AttentionManager.cpp   ./source/ShapeBased3D.cpp
./source/Calmann.cpp            ./source/ShapeModel.cpp
./source/CameraDriver.cpp       ./source/Signature.cpp
./source/Class.cpp              ./source/SignatureDB.cpp
./source/ColorBased.cpp	./source/RelPoseFactory.cpp
./source/ColorModel.cpp	     ./source/SimulatedCamera.cpp
./source/Descriptor.cpp         ./source/Statistics.cpp
./source/DescriptorBased.cpp    ./source/TemplateBased.cpp
./source/Elem.cpp               ./source/TexturedModel.cpp
./source/Filter.cpp             ./source/TrackAlgorithm.cpp
./source/Image.cpp              ./source/VisFinder.cpp
./source/ImageInputSystem.cpp   ./source/VisLearner.cpp
./source/Likelihood.cpp         ./source/XMLTag.cpp
./source/LocateAlgorithm.cpp    ./source/Camera.cpp
./source/Object.cpp             ./source/world.cpp
./source/RefineAlgorithm.cpp	./source/ProveAlgorithm.cpp
./source/PRAShapeVsPointDescr.cpp	./source/RFAColorByShape.cpp
./source/RFAPointDescrByShape.cpp 	./source/DeformShapeModel.cpp
./source/DeformShapeBased.cpp     ./source/LogFile.cpp
./source/SearchParams3d.cpp   ./source/CalTab.cpp
./source/FindCalTab.cpp         ./source/ROSjloComm.cpp
./source/ROSComm.cpp             ./source/ROIGenerator.cpp
./source/ROSCamera.cpp            ./source/SupportingPlaneDetector.cpp
./source/SupportingPlanerDescriptor.cpp ./source/ClusterDetector.cpp
./source/ShapeModelDownloader.cpp   ./source/SimulatedLocate.cpp
./source/NetworkCamera.cpp          ./source/CheckColorClass.cpp
./source/dxfwriter.cpp              ./source/TwoInOneAlg.cpp
./source/File.cpp
)

#if ( LIBXML2_FOUND )
#
#endif ( LIBXML2_FOUND )

#if( Boost_FOUND )
#    if( WIN32 )
#        set( BOOST_LIBS "boost_thread.lib" CACHE STRING "Boost libraries to
#link." )
#    else( WIN32 )
#        set( BOOST_LIBS  CACHE STRING "Boost libraries to
#link." )
#    endif( WIN32 )
#    target_link_libraries( cop ${BOOST_LIBS} )
#endif( Boost_FOUND )
#
#Using player
target_link_libraries( cop ${LIBXML2_LIBRARIES})
#no player
#target_link_libraries(cop ${LIBXML2_LIBRARIES})

set(DEBUG TRUE CACHE BOOL TRUE)

if (DEBUG )
  ADD_DEFINITIONS(-g -DDEFORMSHAPE_AVAILABLE -DSWISS_RANGER_SERVICE -D_DEBUG  -DBOOST_THREAD -DLOGFILE -DBOOST_1_35 ${SWISS_DEFS})
else (DEBUG)
  ADD_DEFINITIONS(-o3 -DDEFORMSHAPE_AVAILABLE -DSWISS_RANGER_SERVICE -DBOOST_THREAD -DLOGFILE -DBOOST_1_35 ${SWISS_DEFS})
endif (DEBUG)
#VERSION Without player
#ADD_DEFINITIONS(-g -D_DEBUG -DHALCONIMG -DBOOST_THREAD)
#ADD_DEFINITIONS(-g -D_DEBUG -DHALCONIMG -DBOOST_THREAD -DLOGFILE)
#VERSION for new dxflib:
#ADD_DEFINITIONS(-g -D_DEBUG -DHALCONIMG -DBOOST_THREAD -DLOGFILE)



SET_TARGET_PROPERTIES(cop PROPERTIES DEFINE_SYMBOL HALCONIMG)
SET_TARGET_PROPERTIES(cop_srv PROPERTIES DEFINE_SYMBOL HALCONIMG)

INSTALL( TARGETS cop
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib )
INSTALL(
    FILES
../include/Action.h              ../include/Object.h
../include/AlgorithmEval.h       ../include/Particle.h
../include/Algorithm.h           ../include/PointDescrModel.h
../include/AlgorithmSelector.h   ../include/YarpComm.h
../include/AttentionManager.h    ../include/PRAShapeVsPointDescr.h
../include/Calmann.h             ../include/ProveAlgorithm.h
../include/CameraDriver.h        ../include/RefineAlgorithm.h
../include/Camera.h              ../include/RelPoseFactory.h
../include/Class.h               ../include/RelPose.h
../include/ColorBased.h          ../include/RFAColorByShape.h
../include/ColorModel.h          ../include/RFAPointDescrByShape.h
../include/CombinedShapeDescr.h  ../include/ShapeBased3D.h
../include/cop.h                 ../include/ShapeModel.h
../include/DeformShapeBased.h    ../include/SignatureDB.h
../include/DeformShapeModel.h    ../include/Signature.h
../include/DescriptorBased.h     ../include/SimulatedCamera.h
../include/Descriptor.h          ../include/Statistics.h
../include/DxfReader.h           ../include/TemplateBased.h
../include/Elem.h                ../include/TexturedModel.h
../include/ElemTypes.h           ../include/TrackAlgorithm.h
../include/Filter.h              ../include/VisFinder.h
../include/Image.h               ../include/VisLearner.h
../include/ImageInputSystem.h    ../include/world.h
../include/Likelihood.h          ../include/XMLTag.h
../include/LocateAlgorithm.h     ../include/RegionOI.h
../include/CalTab.h               ../include/FindCalTab.h
../include/LogFile.h		          ../include/ROSjloComm.h
../include/ROSComm.h              ../include/ROIGenerator.h
../include/ROSCamera.h            ../include/ClusterDetector.h
../include/DxfWriter.h            ../include/SimulatedLocate.h
../include/ShapeModelDownloader.h  ../include/CheckColorClass.h
../include/File.h                  ../include/ColorClass.h
    DESTINATION include/cop )
