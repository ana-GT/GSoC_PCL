cmake_minimum_required( VERSION 2.8 )

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Ceres
find_package( Ceres )
include_directories( ${CERES_INCLUDE_DIRS} )

include_directories( ${CMAKE_SOURCE_DIR}/include )
link_directories( ${CMAKE_SOURCE_DIR}/lib )

# Add tools
add_subdirectory( tools )

