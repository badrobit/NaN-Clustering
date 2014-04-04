/*
 * RegionGrowingNaN.hpp
 *
 *  Created on: Feb 27, 2014
 *      Author: badrobit
 */

#include "RegionGrowingNaN.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
pcl::RegionGrowingNaN<PointT, NormalT>::RegionGrowingNaN()
{
	// TODO Auto-generated constructor stub
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
pcl::RegionGrowingNaN<PointT, NormalT>::~RegionGrowingNaN()
{
	// TODO Auto-generated destructor stub
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingNaN<PointT, NormalT>::extract (std::vector <pcl::PointIndices>& clusters)
{

	clusters_.clear();
	clusters.clear();
	point_neighbours_.clear();
	point_labels_.clear();
	num_pts_in_segment_.clear();
	number_of_segments_ = 0;

	bool segmentation_is_possible = initCompute();
	if ( !segmentation_is_possible )
	{
		deinitCompute();
		return;
	}

	segmentation_is_possible = prepareForSegmentation ();
	if ( !segmentation_is_possible )
	{
		deinitCompute ();
		return;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RegionGrowingNaN<PointT, NormalT>::prepareForSegmentation ()
{
	// if user forgot to pass point cloud or if it is empty
	if ( input_->points.size () == 0 )
		return (false);

  /**
  // if normal/smoothness test is on then we need to check if all needed variables and parameters
  // were correctly initialized
  if (normal_flag_)
  {
    // if user forgot to pass normals or the sizes of point and normal cloud are different
    if ( normals_ == 0 || input_->points.size () != normals_->points.size () )
      return (false);
  }

  // if residual test is on then we need to check if all needed parameters were correctly initialized
  if (residual_flag_)
  {
    if (residual_threshold_ <= 0.0f)
      return (false);
  }

  // if curvature test is on ...
  // if (curvature_flag_)
  // {
  // in this case we do not need to check anything that related to it
  // so we simply commented it
  // }

  // here we check the parameters related to color-based segmentation
  if ( region_neighbour_number_ == 0 || color_p2p_threshold_ < 0.0f || color_r2r_threshold_ < 0.0f || distance_threshold_ < 0.0f )
    return (false);

  // from here we check those parameters that are always valuable
  if (neighbour_number_ == 0)
    return (false);

  // if user didn't set search method
  if (!search_)
    search_.reset (new pcl::search::KdTree<PointT>);

  if (indices_)
  {
    if (indices_->empty ())
      PCL_ERROR ("[pcl::RegionGrowingRGB::prepareForSegmentation] Empty given indices!\n");
    search_->setInputCloud (input_, indices_);
  }
  else
    search_->setInputCloud (input_);

	**/
	return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingNaN<PointT, NormalT>::findPointNeighbours()
{
	int point_number = static_cast<int>( indices_->size() );
	std::vector<int> neighbours;
	std::vector<float> distances;

	point_neighbours_.resize( input_->points.size (), neighbours );
	point_distances_.resize( input_->points.size (), distances );

	for( int i_point = 0; i_point < point_number; i_point++ )
	{
		int point_index = (*indices_)[i_point];
		neighbours.clear();
		distances.clear();
		search_->nearestKSearch( i_point, region_neighbour_number_, neighbours, distances );
		point_neighbours_[point_index].swap( neighbours );
		point_distances_[point_index].swap( distances );
	}
}
