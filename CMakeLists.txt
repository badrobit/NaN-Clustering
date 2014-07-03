CMAKE_MINIMUM_REQUIRED( VERSION 2.8 FATAL_ERROR )
PROJECT( NaN-Clustering )

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    # Other flags
)

#.....: PCL Stuff .................................................................................:
SET( PCL_DIR "${HOME}/pcl-trunk/build/PCLConfig.cmake" )
FIND_PACKAGE( PCL 1.7 REQUIRED PATHS "${HOME}/pcl-trunk/build/PCLConfig.cmake" )

INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} 
                     ${PROJECT_SOURCE_DIR}/include )
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

#.....: Libraries .................................................................................:
ADD_LIBRARY( HelperFunctions src/HelperFunctions.cpp )
SET_TARGET_PROPERTIES( HelperFunctions PROPERTIES LINKER_LANGUAGE CXX )
TARGET_LINK_LIBRARIES( HelperFunctions ${PCL_LIBRARIES} )

ADD_LIBRARY( NaNClusterComparator include/NaNClusterComparator.h )
SET_TARGET_PROPERTIES( NaNClusterComparator PROPERTIES LINKER_LANGUAGE CXX )
TARGET_LINK_LIBRARIES( NaNClusterComparator ${PCL_LIBRARIES} )

#.....: Executable ................................................................................:
ADD_EXECUTABLE( NaNClusteringDemo test/NaNClusteringDemo.cpp )
TARGET_LINK_LIBRARIES( NaNClusteringDemo NaNClusterComparator HelperFunctions )