# Add test 1 
link_directories( ${CMAKE_SOURCE_DIR}/lib )

add_executable( testSegmented fitSegmentedPointcloud.cpp )
target_link_libraries( testSegmented  fitterSQ ${PCL_LIBRARIES} )
set_target_properties( testSegmented PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

 
