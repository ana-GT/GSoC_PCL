cmake_minimum_required( VERSION 2.8 )

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Ceres
find_package( Ceres )

include_directories( ${CMAKE_SOURCE_DIR}/include )
link_directories( ${CMAKE_SOURCE_DIR}/lib )

# SQ_utils library
add_library( SQ_utils src/SQ_utils.cpp src/analytic_equations.cpp )
target_link_libraries( SQ_utils ${PCL_LIBRARIES} ${CERES_LIBRARIES} pthread )
set_target_properties( SQ_utils PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# SQ_fitter library
add_library( SQ_fitter src/SQ_fitter.cpp )
target_link_libraries( SQ_fitter SQ_utils ${PCL_LIBRARIES} ${CERES_LIBRARIES} pthread )
set_target_properties( SQ_fitter PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Run test
add_executable( SQ_fitter_test1 tests/SQ_fitter_test1.cpp )
target_link_libraries( SQ_fitter_test1 SQ_fitter ${PCL_LIBRARIES} ${CERES_LIBRARIES} pthread )
set_target_properties( SQ_fitter_test1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( cloud_basicOperations tests/cloud_basicOperations.cpp )
target_link_libraries( cloud_basicOperations ${PCL_LIBRARIES} )
set_target_properties( cloud_basicOperations PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( SQ_utils_SE_test tests/SQ_utils_SE_test.cpp )
target_link_libraries( SQ_utils_SE_test SQ_utils ${PCL_LIBRARIES} )
set_target_properties( SQ_utils_SE_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( sampleSQ tests/sampleSQ.cpp )
target_link_libraries( sampleSQ SQ_utils ${PCL_LIBRARIES} )
set_target_properties( sampleSQ PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

