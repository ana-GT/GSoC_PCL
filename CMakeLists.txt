cmake_minimum_required( VERSION 2.8 )

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Ceres
find_package( Ceres )

# Add previous stuff
add_subdirectory( tools )

include_directories( ${CMAKE_SOURCE_DIR}/tools/include )

# Run test
add_executable( ceresSolver src/ceresSolver.cpp )
target_link_libraries( ceresSolver SQ_sampler ${PCL_LIBRARIES} ${CERES_LIBRARIES} pthread )
set_target_properties( ceresSolver PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )



