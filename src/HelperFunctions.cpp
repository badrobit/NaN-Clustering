#include "HelperFunctions.h"

PointCloud ReadFromFile( std::string file_name, file_type type )
{
	PointCloud::Ptr cloud( new PointCloud );

	if( type == PCD )
	{
		if( pcl::io::loadPCDFile<PointT>( file_name, *cloud ) == -1 ) //* load the file
		{
			PCL_ERROR( "Couldn't read file: %s", file_name.c_str() );
		}
	}
	else if( type == PLY )
	{
		if( pcl::io::loadPLYFile<PointT>( file_name, *cloud ) == -1 ) //* load the file
		{
			PCL_ERROR( "Couldn't read file: %s", file_name.c_str() );
		}
	}

  	PCL_INFO( "Loaded %d data points from %s \n", cloud->width * cloud->height, file_name.c_str() );
  	return *cloud;
}

int SaveToFile( PointCloud input_cloud, std::string file_name, file_type type )
{
	int return_value = 0;

	if( type == PCD )
	{
		pcl::io::savePLYFileASCII( file_name + ".pcd", input_cloud );
		std::cout << "Saved " << input_cloud.size() << " datapoints to " << file_name << ".ply" << std::endl;
		return true;
	}
	else if( type == PLY )
	{

	}
	else if( type == BOTH )
	{

	}
}
