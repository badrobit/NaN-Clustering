/*
 * RegionGrowingNaN.h
 *
 *  Created on: Feb 27, 2014
 *      Author: badrobit
 */

#ifndef PCL_REGION_GROWING_NAN_H_
#define PCL_REGION_GROWING_NAN_H_

#include <pcl/segmentation/region_growing.h>

namespace pcl
{
	template< typename PointT, typename NormalT = pcl::Normal >
	class PCL_EXPORTS RegionGrowingNaN : public RegionGrowing<PointT, NormalT>
	{
		public:
			using RegionGrowing<PointT, NormalT>::input_;
			using RegionGrowing<PointT, NormalT>::indices_;
			using RegionGrowing<PointT, NormalT>::initCompute;
			using RegionGrowing<PointT, NormalT>::deinitCompute;
			//using RegionGrowing<PointT, NormalT>::normals_;
			//using RegionGrowing<PointT, NormalT>::normal_flag_;
			//using RegionGrowing<PointT, NormalT>::curvature_flag_;
			//using RegionGrowing<PointT, NormalT>::residual_flag_;
			//using RegionGrowing<PointT, NormalT>::residual_threshold_;
			//using RegionGrowing<PointT, NormalT>::neighbour_number_;
			using RegionGrowing<PointT, NormalT>::search_;
			using RegionGrowing<PointT, NormalT>::min_pts_per_cluster_;
			using RegionGrowing<PointT, NormalT>::max_pts_per_cluster_;
			//using RegionGrowing<PointT, NormalT>::smooth_mode_flag_;
			//using RegionGrowing<PointT, NormalT>::theta_threshold_;
			//using RegionGrowing<PointT, NormalT>::curvature_threshold_;
			using RegionGrowing<PointT, NormalT>::point_neighbours_;
			using RegionGrowing<PointT, NormalT>::point_labels_;
			using RegionGrowing<PointT, NormalT>::num_pts_in_segment_;
			using RegionGrowing<PointT, NormalT>::clusters_;
			using RegionGrowing<PointT, NormalT>::number_of_segments_;
			//using RegionGrowing<PointT, NormalT>::applySmoothRegionGrowingAlgorithm;
			//using RegionGrowing<PointT, NormalT>::assembleRegions;

		public:

			RegionGrowingNaN();

			virtual ~RegionGrowingNaN();

			virtual void extract(std::vector <pcl::PointIndices>& clusters);

		private:

			bool prepareForSegmentation();

			void findPointNeighbours();

	}; /* class region_growing_nan */
} /* namespace pcl */

#endif /* PCL_REGION_GROWING_NAN_H_ */
