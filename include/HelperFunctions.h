#ifndef HELPERFUNCTIONS_H_
#define HELPERFUNCTIONS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

/**
 *
 */
enum file_type{ PCD, PLY, BOTH };

/**
* \typedef PointT
* \brief Have all points in the project represented by pcl::PointXYZRGB
* */
typedef pcl::PointXYZRGBA PointT;

/**
* \typedef PointCloud
* \brief Have all pointclouds used in the project made up from \ref PointT
*
* \details This representation is used to save time and type matching errors while passing around
* and working with point clouds.
*/
typedef pcl::PointCloud<PointT> PointCloud;

/**
 * \brief Reads PointCloud data in from a file.
 *
 * @param file_name The name of the PCD/PLY file that is to be loaded.
 * @param extension The extension of the file to be loaded so that we can use the proper reader.
 *
 * @return This returns a PointCloud made from the input file.
 */
PointCloud ReadFromFile( std::string file_name, file_type type );


/**
 *
 * @param file_name
 * @param type
 * @return
 */
int SaveToFile( PointCloud input_cloud, std::string file_name, file_type type );

#endif /* HELPERFUNCTIONS_H_ */
