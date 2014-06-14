cmake_minimum_required( VERSION 2.8 )

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Ceres
find_package( Ceres )

# Add previous stuff
#add_subdirectory( tools )
#include_directories( ${CMAKE_SOURCE_DIR}/tools/include )

include_directories( ${CMAKE_SOURCE_DIR}/include )
link_directories( ${CMAKE_SOURCE_DIR}/lib )

# SQ_fitter library
add_library( SQ_fitter src/SQ_fitter.cpp )
target_link_libraries( SQ_fitter ${PCL_LIBRARIES} ${CERES_LIBRARIES} pthread )
set_target_properties( SQ_fitter PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Run test
add_executable( SQ_fitter_test1 tests/SQ_fitter_test1.cpp )
target_link_libraries( SQ_fitter_test1 SQ_fitter ${PCL_LIBRARIES} ${CERES_LIBRARIES} pthread )
set_target_properties( SQ_fitter_test1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )



