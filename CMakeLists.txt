cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project( NaN-Clustering )

find_package( PCL 1.7 REQUIRED )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR} )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

include_directories( 
	${PROJECT_SOURCE_DIR}/include
	${PCL_INCLUDE_DIRS} )

link_directories( ${PCL_LIBRARY_DIRS} )

add_definitions( ${PCL_DEFINITIONS} )

#.....: Libraries .................................................................................:
add_library( HelperFunctions src/HelperFunctions.cpp ) 
target_link_libraries( HelperFunctions ${PCL_LIBRARIES} )

#.....: Executable ................................................................................:
ADD_EXECUTABLE( NaN-Clustering main.cpp )
target_link_libraries( NaN-Clustering HelperFunctions )