# CMake for helper tools in SQ project
project( tools )

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# include
include_directories( ${CMAKE_SOURCE_DIR}/tools/include )

# Libraries
add_library( SQ_sampler src/SQ_sampler.cpp )
target_link_libraries( SQ_sampler ${PCL_LIBRARIES} )
set_target_properties( SQ_sampler PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

link_directories( ${CMAKE_SOURCE_DIR}/lib )

# Test
add_executable( SQ_test tests/SQ_test.cpp )
target_link_libraries( SQ_test SQ_sampler ${PCL_LIBRARIES} )
set_target_properties( SQ_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Minimizer test
add_library( minimizer tests/minimizer.cpp )
target_link_libraries( minimizer ${PCL_LIBRARIES} )
set_target_properties( minimizer PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

add_executable( minimizeTest tests/minimizeTest.cpp )
target_link_libraries( minimizeTest minimizer ${PCL_LIBRARIES} )
set_target_properties( minimizeTest PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

