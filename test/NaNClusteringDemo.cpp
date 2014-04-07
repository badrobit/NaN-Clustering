#include <NaNClusterComparator.h>
#include <HelperFunctions.h>

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

 int main( int argc, char *argv[] )
 {
   std::string incloudfile = argv[1];
   std::string outcloudfile = argv[2];

   // Load cloud
   pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT> );
   pcl::io::loadPCDFile( incloudfile.c_str (), *cloud );

   pcl::PointCloud<PointT> outcloud;

	// // Estimate Normals
 //   pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
 //   pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
 //   ne.setInputCloud (cloud);
 //   ne.compute (*normal_cloud);

 //   // Segment Planes
 //   double mps_start = pcl::getTime ();
 //   pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
 //   std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
 //   std::vector<pcl::ModelCoefficients> model_coefficients;
 //   std::vector<pcl::PointIndices> inlier_indices;
 //   pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
 //   std::vector<pcl::PointIndices> label_indices;
 //   std::vector<pcl::PointIndices> boundary_indices;
 //   mps.setInputNormals (normal_cloud);
 //   mps.setInputCloud (cloud);
 //   mps.segment (regions);//, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
 //   double mps_end = pcl::getTime ();
 //   std::cout << "MPS+Refine took: " << double(mps_end - mps_start) << std::endl;

   //pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr euclidean_comparator_;
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


   // if (use_clustering_ && regions.size () > 0)
   // {
   // std::vector<bool> plane_labels;
   // plane_labels.resize (label_indices.size (), false);
   // for (size_t i = 0; i < label_indices.size (); i++)
   // {
   // if (label_indices[i].indices.size () > 10000)
   // {
   // plane_labels[i] = true;
   // }
   // }

   
   // euclidean_cluster_comparator_->setLabels (labels);
   // euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
   // euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);

   // pcl::PointCloud<pcl::Label> euclidean_labels;
   // std::vector<pcl::PointIndices> euclidean_label_indices;
   // pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
   // euclidean_segmentation.setInputCloud (cloud);
   // euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

   // for (size_t i = 0; i < euclidean_label_indices.size (); i++)
   // {
   // if (euclidean_label_indices[i].indices.size () > 1000)
   // {
   // pcl::PointCloud<PointT> cluster;
   // pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,cluster);
   // clusters.push_back (cluster);
   // }
   // }

   // PCL_INFO ("Got %d euclidean clusters!\n", clusters.size ());
   // } 

   // Save filtered output
   //pcl::io::savePCDFile (outcloudfile.c_str (), outcloud);
   return (0);
 }
