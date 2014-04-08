#include <NaNClusterComparator.h>
#include <HelperFunctions.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>


class SimpleOpenNIViewer
{
   public:
      SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

      void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
      {
         if (!viewer.wasStopped())
         viewer.showCloud (cloud);
         //std::cerr << "PointCloud Width: " << cloud->width << " PointCloud Height: " << cloud->height << "\n"; 
         //pcl::io::savePCDFileASCII( "test01.pcd", *cloud );

         std::cerr << "PointCloud Width: " << cloud->width << " PointCloud Height: " << cloud->height << "\n"; 
         std::cerr << "PointCloud is Organized: " << cloud->isOrganized() << std::endl; 
         std::cerr << "PointCloud Contains NaN: " << !cloud->is_dense << std::endl; 

         pcl::NaNClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr nan_comparator; 
         nan_comparator.reset( new pcl::NaNClusterComparator<PointT, pcl::Normal, pcl::Label>() );

         //Segment Objects
         pcl::PointCloud<PointT>::CloudVectorType clusters;

         nan_comparator->setInputCloud( cloud );
         pcl::PointCloud<pcl::Label> nan_labels;
         std::vector<pcl::PointIndices> nan_label_indices;

         pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> nan_segmentation( nan_comparator );
         nan_segmentation.setInputCloud( cloud );
         nan_segmentation.segment( nan_labels, nan_label_indices );

         for (size_t i = 0; i < nan_label_indices.size (); i++)
         {
            if( nan_label_indices[i].indices.size() > 50 )
            {
               pcl::PointCloud<PointT> cluster;
               pcl::copyPointCloud( *cloud, nan_label_indices[i].indices,cluster );
               clusters.push_back( cluster );
            }
         }

         PCL_INFO( "Got %d nan clusters!\n", clusters.size() );

      }

      void run ()
      {
         pcl::Grabber* interface = new pcl::OpenNIGrabber();

         boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

         interface->registerCallback (f);

         interface->start ();

         while (!viewer.wasStopped())
         {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
         }

         interface->stop ();
      }

      pcl::visualization::CloudViewer viewer;
};


int main( int argc, char *argv[] )
{
   std::string incloudfile = argv[1];
   //std::string outcloudfile = argv[2];

   SimpleOpenNIViewer v; 
   v.run(); 

   // Load cloud
   // pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT> );
   // pcl::io::loadPCDFile( incloudfile.c_str (), *cloud );

   // std::cerr << "PointCloud Width: " << cloud->width << " PointCloud Height: " << cloud->height << "\n"; 
   // std::cerr << "PointCloud is Organized: " << cloud->isOrganized() << std::endl; 
   // std::cerr << "PointCloud Contains NaN: " << !cloud->is_dense << std::endl;; 

   // pcl::PointCloud<PointT> outcloud;

   // pcl::NaNClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr nan_comparator; 
   // nan_comparator.reset( new pcl::NaNClusterComparator<PointT, pcl::Normal, pcl::Label>() );

   // //Segment Objects
   // pcl::PointCloud<PointT>::CloudVectorType clusters;

   // nan_comparator->setInputCloud( cloud );
   // pcl::PointCloud<pcl::Label> nan_labels;
   // std::vector<pcl::PointIndices> nan_label_indices;

   // pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> nan_segmentation( nan_comparator );
   // nan_segmentation.setInputCloud( cloud );
   // nan_segmentation.segment( nan_labels, nan_label_indices );

   // for (size_t i = 0; i < nan_label_indices.size (); i++)
   // {
   //    if( nan_label_indices[i].indices.size() > 50 )
   //    {
   //       pcl::PointCloud<PointT> cluster;
   //       pcl::copyPointCloud( *cloud, nan_label_indices[i].indices,cluster );
   //       clusters.push_back( cluster );
   //    }
   // }

   // PCL_INFO( "Got %d nan clusters!\n", clusters.size() );

   return (0);
}
