#include "HelperFunctions.h"

int main (int argc, char** argv)
{
	if (argc < 2 || pcl::console::find_switch (argc, argv, "--help"))
	{
		pcl::console::print_error( "Usage: %s\n"
								   "\t\t--pcd-file <pcd-file>\n"
								   "\t\t--ply-file <ply-file>\n"
							   	   "\t\t--kinect\n"
								   "\t\t--save-output\n"
							   	   , argv[0] );
	return (1);
	}

	bool option_load_pcd = pcl::console::find_switch (argc, argv, "--pcd-file");
	bool option_load_ply = pcl::console::find_switch (argc, argv, "--ply-file");

	std::string pcd_file_name;
	if( option_load_pcd )
	{
		pcl::console::parse( argc, argv, "--pcd-file", pcd_file_name );
		ReadFromFile( pcd_file_name, 0 );
	}

	std::string ply_file_name;
	if( option_load_ply )
	{
		pcl::console::parse( argc, argv, "--pcd-file", pcd_file_name );
		ReadFromFile( pcd_file_name, 1 );
	}
}


