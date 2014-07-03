#include <NaNClusterComparator.h>
#include <HelperFunctions.h>

#include <pcl/io/png_io.h>
#include <pcl/io/openni_grabber.h>

#include <pcl/common/time.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    ++count; \
    if (pcl::getTime() - last >= 1.0) \
    { \
      double now = pcl::getTime (); \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif


boost::mutex depth_image_mutex, ir_image_mutex, image_mutex;
boost::shared_ptr<openni_wrapper::Image> image_ptr;

class SimpleOpenNIViewer
{
   public:
      SimpleOpenNIViewer(): viewer( "PCL OpenNI Viewer" ) {}

      void ProjectAndCompare( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud )
      {
         // Create a planar coefficients with X=1 Y=Z=0 (project into the YZ plane)
         pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
         coefficients->values.resize (4);
         coefficients->values[0] = coefficients->values[1] = 0; 
         coefficients->values[2] = 1.0; 
         coefficients->values[3] = 0;

         // Create the filtering object
         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGBA>);
         pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
         proj.setModelType (pcl::SACMODEL_PLANE);
         proj.setInputCloud (cloud);
         proj.setModelCoefficients (coefficients);
         proj.filter (*cloud_projected);

         //std::cerr << "PointCloud Width: " << cloud_projected->width << " PointCloud Height: " << cloud_projected->height << "\n"; 

         //pcl::io::savePNGFile<pcl::PointXYZRGBA>( "test", *cloud_projected, "rgb" );
         //savePNGFile( "original.png", cloud ); 
         //savePNGFile( "processed.png", cloud_projected ); 
         //viewer.showCloud( cloud_projected );
         //point_cloud_to_png( cloud_projected ); 
      }

      void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
      {
         if (!viewer.wasStopped())
         viewer.showCloud (cloud);

         uint8_t r = 255, g = 0, b = 0;    // Example: Red color
         uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

         //std::cerr << "PointCloud Width: " << cloud->width << " PointCloud Height: " << cloud->height << "\n"; 
         //std::cerr << "PointCloud is Organized: " << cloud->isOrganized() << std::endl; 
         //std::cerr << "PointCloud Contains NaN: " << !cloud->is_dense << std::endl; 

         pcl::NaNClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr nan_comparator; 
         nan_comparator.reset( new pcl::NaNClusterComparator<PointT, pcl::Normal, pcl::Label>() );

         //Segment Objects
         pcl::PointCloud<PointT>::CloudVectorType clusters;

         nan_comparator->setInputCloud( cloud );
         pcl::PointCloud<pcl::Label> nan_labels;
         std::vector<pcl::PointIndices> nan_label_indices;

         FPS_CALC( "NaN-Clustering" ); 
         pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> nan_segmentation( nan_comparator );
         nan_segmentation.setInputCloud( cloud );
         nan_segmentation.segment( nan_labels, nan_label_indices );

         pcl::PointCloud<PointT> output;
         pcl::copyPointCloud( *cloud, output ); 

         for (size_t i = 0; i < nan_label_indices.size (); i++)
         {
            if( nan_label_indices[i].indices.size() > 50 )
            {
               std::cout << "NaN Cluster " << i << " size: " << nan_label_indices[i].indices.size() << std::endl; 
               pcl::PointCloud<PointT> cluster;
               pcl::copyPointCloud( *cloud, nan_label_indices[i].indices,cluster );
               clusters.push_back( cluster );
               
               for( size_t x = 0; x < nan_label_indices[i].indices.size(); x ++ )
               {
                  //std::cout << "Point color: " << cloud->points.at( nan_label_indices[i].indices[x] ).rgb << std::endl;
                  output.points.at( nan_label_indices[i].indices[x] ).rgb = *reinterpret_cast<float*>(&rgb);
               }
            }
         }

         PCL_INFO( "Got %d nan clusters!\n", clusters.size() );
         //ProjectAndCompare( cloud ); 
         
            std::stringstream ss, s2;
            ss << "Original_" << itr << ".png"; 
            s2 << "NaNClusters" << itr << ".png"; 
            point_cloud_to_png( ss.str(), cloud ); 
            point_cloud_to_png( s2.str(), output.makeShared() );   
            itr++; 
      }

      void image_cb_( const boost::shared_ptr<openni_wrapper::Image>& image )
      {
         if( image_mutex.try_lock() )
         {
            image_ptr = image;
            image_mutex.unlock ();
            //savePNGFile( "original.png", image ); 
            //received_new_image = true;
         }
      }

      void point_cloud_to_png( std::string file_name,  const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud )
      {
         std::vector<unsigned char> data(cloud->width * cloud->height * 3);

         for( size_t i = 0; i < cloud->points.size(); ++i )
         {
           data[i*3 + 0] = cloud->points[i].r;
           data[i*3 + 1] = cloud->points[i].g;
           data[i*3 + 2] = cloud->points[i].b;        
         }
         pcl::io::saveRgbPNGFile( file_name, &data[0], cloud->width, cloud->height );
      }

      void run ()
      {
         pcl::Grabber* interface = new pcl::OpenNIGrabber();

         boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f_cloud =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

         boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > f_image =
         boost::bind( &SimpleOpenNIViewer::image_cb_, this, _1);
  
         interface->registerCallback( f_cloud );
         interface->registerCallback( f_image ); 

         interface->start ();

         while (!viewer.wasStopped())
         {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
         }

         interface->stop ();
      }

      pcl::visualization::CloudViewer viewer;
      int itr = 0; 
};


int main( int argc, char *argv[] )
{
   SimpleOpenNIViewer v; 
   v.run(); 
   return (0);
}
