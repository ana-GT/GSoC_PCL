# CMake for helper tools in SQ project
project( tools Fortran CXX )

set( CMAKE_Fortran_FLAGS "-c -g " )

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# include and link
include_directories( ${PROJECT_SOURCE_DIR}/include )
link_directories( ${CMAKE_SOURCE_DIR}/lib )

# Libraries
add_library( SQ_sampler src/SQ_sampler.cpp )
target_link_libraries( SQ_sampler ${PCL_LIBRARIES} )
set_target_properties( SQ_sampler PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Generate Fortran file
add_custom_command( 
	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/JH.f 
	COMMAND maxima --very-quiet -b ${PROJECT_SOURCE_DIR}/src/JH.mac )
add_library( JH ${CMAKE_CURRENT_BINARY_DIR}/JH.f )
set_target_properties( JH PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Minimizer test
add_library( minimizerDraft src/minimizer.cpp )
target_link_libraries( minimizerDraft JH ${PCL_LIBRARIES} )
set_target_properties( minimizerDraft PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# SQ_fitter
add_library( SQ_fitter src/SQ_fitter.cpp )
target_link_libraries( SQ_fitter ${PCL_LIBRARIES} )
set_target_properties( SQ_fitter PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# SQ_fitter test
add_executable( SQFitter_test tests/SQ_fitter_test.cpp )
target_link_libraries( SQFitter_test SQ_fitter ${PCL_LIBRARIES}  )
set_target_properties( SQFitter_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )



# Test
add_executable( SQ_test tests/SQ_test.cpp )
target_link_libraries( SQ_test SQ_sampler ${PCL_LIBRARIES} )
set_target_properties( SQ_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Minimizer test
add_executable( minimizeDraftTest tests/minimizeDraftTest.cpp src/minimizer.cpp )
target_link_libraries( minimizeDraftTest JH ${PCL_LIBRARIES}  )
set_target_properties( minimizeDraftTest PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

