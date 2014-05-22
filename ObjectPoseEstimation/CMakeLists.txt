cmake_minimum_required( VERSION 2.8 )

include_directories( ${CMAKE_SOURCE_DIR}/include )

#Eigen
include( FindPkgConfig )
pkg_check_modules( EIGEN eigen3 )
include_directories( ${EIGEN_INCLUDE_DIRS} )

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Setting stuff
set( fitter_src src/InertiaCalculations.cpp src/Minimization.cpp
  src/OPESettings.cpp src/OPEMain.cpp src/ObjectPoseEstimator.cpp src/Plane.cpp 
  src/PointCloudCapture.cpp src/OPEUtils.cpp src/SQFitting.cpp )

set( fitter_hdr include/OPECommon.h include/OPESettings.h include/EigenSolver.h 
  include/InertiaCalculations.h include/Minimization.h include/ObjectPoseEstimator.h
  include/Plane.h include/PointCloudCapture.h include/SQFitting.h include/SQTypes.h include/TableObjectDetector.h
  include/TableObjectDetector.hpp include/TableObjectModeler.h include/OPEUtils.h )

# Add library
add_library( fitterSQ ${fitter_src} ${fitter_hdr} )
target_link_libraries( fitterSQ ${PCL_LIBRARIES} ) 
set_target_properties( fitterSQ PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Add test
add_subdirectory( tests )

