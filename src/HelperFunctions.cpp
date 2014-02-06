#include "HelperFunctions.h"

PointCloud ReadFromFile( std::string file_name, int extension )
{
	PointCloud::Ptr cloud( new PointCloud );

	if( extension == 0 )
	{
		if( pcl::io::loadPCDFile<PointT>( file_name, *cloud ) == -1 ) //* load the file
		{
			PCL_ERROR( "Couldn't read file: %s", file_name.c_str() );
		}
	}
	else if( extension == 1 )
	{
		if( pcl::io::loadPLYFile<PointT>( file_name, *cloud ) == -1 ) //* load the file
		{
			PCL_ERROR( "Couldn't read file: %s", file_name.c_str() );
		}
	}

  	PCL_INFO( "Loaded %d data points from %s \n", cloud->width * cloud->height, file_name.c_str() );
  	return *cloud;
}
