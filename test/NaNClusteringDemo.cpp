#include <NaNClusterComparator.h>
#include <HelperFunctions.h>

 int main( int argc, char *argv[] )
 {
   std::string incloudfile = argv[1];
   std::string outcloudfile = argv[2];

   // Load cloud
   pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT> );
   pcl::io::loadPCDFile( incloudfile.c_str (), *cloud );

   pcl::PointCloud<PointT> outcloud;

	// Estimate Normals
   pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
   pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
   ne.setInputCloud (cloud);
   ne.compute (*normal_cloud);

   // Save filtered output
   pcl::io::savePCDFile (outcloudfile.c_str (), outcloud);
   return (0);
 }
