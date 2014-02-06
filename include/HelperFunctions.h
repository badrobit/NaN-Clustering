#ifndef HELPERFUNCTIONS_H_
#define HELPERFUNCTIONS_H_

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>

/**
* \typedef PointT
* \brief Have all points in the project represented by pcl::PointXYZRGB
* */
typedef pcl::PointXYZRGB PointT;

/**
* \typedef PointCloud
* \brief Have all pointclouds used in the project made up from \ref PointT
*
* \details This representation is used to save time and type matching errors while passing around
* and working with point clouds.
*/
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud ReadFromFile( std::string file_name, int extension );

#endif /* HELPERFUNCTIONS_H_ */
