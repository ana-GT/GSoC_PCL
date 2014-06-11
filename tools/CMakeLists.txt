# CMake for helper tools in SQ project
project( tools Fortran CXX )

set( CMAKE_Fortran_FLAGS "-c -g " )
set( CMAKE_CXX_FLAGS "-std=gnu++11" )

# Eigen
find_package( PkgConfig )
pkg_check_modules( EIGEN eigen3 )
include_directories( ${EIGEN_INCLUDE_DIRS} )

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# include and link
include_directories( ${PROJECT_SOURCE_DIR}/include )
link_directories( ${CMAKE_SOURCE_DIR}/lib )

# ******************************************************
# Libraries
# ******************************************************

# Utilities
add_library( SQ_utils src/SQ_utils.cpp )
target_link_libraries( SQ_utils ${PCL_LIBRARIES} )
set_target_properties( SQ_utils PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Sample SQ
add_library( SQ_sampler src/SQ_sampler.cpp )
target_link_libraries( SQ_sampler SQ_utils ${PCL_LIBRARIES} )
set_target_properties( SQ_sampler PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Minimizer
add_library( SQ_minimizer src/minimizer.cpp src/matlab_equations.cpp )
target_link_libraries( SQ_minimizer SQ_sampler ${PCL_LIBRARIES} )
set_target_properties( SQ_minimizer PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )


# Fit a SQ
add_library( SQ_fitter src/SQ_fitter.cpp )
target_link_libraries( SQ_fitter  SQ_minimizer  SQ_utils  SQ_sampler ${PCL_LIBRARIES} )
set_target_properties( SQ_fitter PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# ******************************************************


# ******************************************************
# Executable Tests
# ******************************************************

# Minimizer test
add_executable( minimizer_test tests/minimizer_test.cpp )
target_link_libraries( minimizer_test SQ_minimizer ${PCL_LIBRARIES}  )
set_target_properties( minimizer_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Sample
add_executable( sampler_test tests/sampler_test.cpp )
target_link_libraries( sampler_test  SQ_utils SQ_sampler ${PCL_LIBRARIES} )
set_target_properties( sampler_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Sample test 2
add_executable( sampler_test2 tests/sampler_test2.cpp )
target_link_libraries( sampler_test2  SQ_utils SQ_sampler ${PCL_LIBRARIES} )
set_target_properties( sampler_test2 PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# Fitting SQ
add_executable( fitting_test tests/fitting_test.cpp )
target_link_libraries( fitting_test SQ_fitter SQ_utils  SQ_sampler SQ_minimizer ${PCL_LIBRARIES}  )
set_target_properties( fitting_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( fitting_test2 tests/fitting_test2.cpp )
target_link_libraries( fitting_test2  SQ_fitter SQ_utils SQ_sampler SQ_minimizer ${PCL_LIBRARIES}  )
set_target_properties( fitting_test2 PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# ******************************************************
