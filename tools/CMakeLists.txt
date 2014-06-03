# CMake for helper tools in SQ project
project( tools Fortran CXX )

set( CMAKE_Fortran_FLAGS "-c -g " )
set( CMAKE_CXX_FLAGS "-std=gnu++11" )

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
add_library( minimizer_SQ src/minimizer.cpp )
target_link_libraries( minimizer_SQ SQ_sampler JH ${PCL_LIBRARIES} )
set_target_properties( minimizer_SQ PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )


# Fit a SQ
add_library( SQ_fitter src/SQ_fitter.cpp )
target_link_libraries( SQ_fitter  minimizer SQ_sampler SQ_utils JH ${PCL_LIBRARIES} )
set_target_properties( SQ_fitter PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Generate Fortran math for symbolic equations
add_custom_command( 
	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/JH.f 
	COMMAND maxima --very-quiet -b ${PROJECT_SOURCE_DIR}/src/JH.mac )
add_library( JH ${CMAKE_CURRENT_BINARY_DIR}/JH.f )
set_target_properties( JH PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# ******************************************************


# ******************************************************
# Executable Tests
# ******************************************************

# Fitting SQ
#add_executable( fitting_test tests/fitting_test.cpp )
#target_link_libraries( fitting_test SQ_fitter SQ_sampler minimizer JH ${PCL_LIBRARIES}  )
#set_target_properties( fitting_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( fitting_test2 tests/fitting_test2.cpp )
target_link_libraries( fitting_test2 SQ_fitter JH ${PCL_LIBRARIES}  )
set_target_properties( fitting_test2 PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Sample
add_executable( sampler_test tests/sampler_test.cpp )
target_link_libraries( sampler_test SQ_sampler SQ_utils ${PCL_LIBRARIES} )
set_target_properties( sampler_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Minimizer test
add_executable( minimizer_test tests/minimizer_test.cpp )
target_link_libraries( minimizer_test minimizer_SQ JH ${PCL_LIBRARIES}  )
set_target_properties( minimizer_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# ******************************************************