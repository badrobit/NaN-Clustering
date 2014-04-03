cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project( NaN-Clustering CXX )

find_package( PCL 1.7 REQUIRED )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR} )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

include_directories( 
	${PROJECT_SOURCE_DIR}/include
	${PCL_INCLUDE_DIRS} )

link_directories( ${PCL_LIBRARY_DIRS} )

add_definitions( ${PCL_DEFINITIONS} )

#SET(GCC_COVERAGE_COMPILE_FLAGS "-fpermissive")
#add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

#.....: Libraries .................................................................................:
#add_library( HelperFunctions src/HelperFunctions.cpp ) 
#target_link_libraries( HelperFunctions ${PCL_LIBRARIES} )

add_library( RegionGrowingNaN src/RegionGrowingNaN.cpp )
target_link_libraries( RegionGrowingNaN ${PCL_LIBRARIES} pcl_common )

#.....: Executable ................................................................................:
#ADD_EXECUTABLE( NaN-Clustering main.cpp )
#target_link_libraries( NaN-Clustering HelperFunctions )