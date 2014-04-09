/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	  notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PCL_SEGMENTATION_NAN_CLUSTER_COMPARATOR_H_
#define PCL_SEGMENTATION_NAN_CLUSTER_COMPARATOR_H_

#include <pcl/segmentation/boost.h>
#include <pcl/segmentation/comparator.h>

namespace pcl
{
	/**
	 * \brief EuclideanClusterComparator is a comparator used for finding clusters supported by planar surfaces.
	 * This needs to be run as a second pass after extracting planar surfaces, using MultiPlaneSegmentation for example.
	 *
	 * \author Matthew Roscoe (mat.roscoe@unb.ca)
	 */
	template<typename PointT, typename PointNT, typename PointLT>
	class NaNClusterComparator : public Comparator<PointT>
	{
		public:
			typedef typename Comparator<PointT>::PointCloud PointCloud;
			typedef typename Comparator<PointT>::PointCloudConstPtr PointCloudConstPtr;

			typedef typename pcl::PointCloud<PointNT> PointCloudN;
			typedef typename PointCloudN::Ptr PointCloudNPtr;
			typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;

			typedef typename pcl::PointCloud<PointLT> PointCloudL;
			typedef typename PointCloudL::Ptr PointCloudLPtr;
			typedef typename PointCloudL::ConstPtr PointCloudLConstPtr;

			typedef boost::shared_ptr<NaNClusterComparator<PointT, PointNT, PointLT> > Ptr;
			typedef boost::shared_ptr<const NaNClusterComparator<PointT, PointNT, PointLT> > ConstPtr;

			using pcl::Comparator<PointT>::input_;

			/**
			 * \brief Empty constructor for NaNClusterComparator.
			 */
			NaNClusterComparator()
			{
			}

			/**
			 * \brief Destructor for NaNClusterComparator
			 */
			~NaNClusterComparator(){};

			/**
			 * \brief Provide a pointer to the input point cloud.
			 * \param[in] Input Point Cloud.
			 */
			virtual void setInputCloud( const PointCloudConstPtr& cloud )
			{
				if( !cloud->is_dense )
				{
					input_ = cloud;	
				}
				else
				{
					std::cerr << "You must provide a point cloud that contains NaN values." << std::endl; 
				}				
			}

			/**
			 * \brief Set label cloud
			 * \param[in] labels The label cloud
			 */
			void setLabels( PointCloudLPtr& labels )
			{
				labels_ = labels;
			}


			virtual bool compare( int idx1, int idx2 ) const
			{
				bool pt1_is_nan, pt2_is_nan = false;  
				//std::cout << "I'm being called!\n"; 

				if( !pcl_isfinite( input_->points[ idx1 ].x ) || !pcl_isfinite( input_->points[ idx1 ].y ) || !pcl_isfinite( input_->points[ idx1 ].z ) )
				{
					//std::cout << "Pt1 is NaN!\n"; 
					pt1_is_nan = true; 
				}
				if( !pcl_isfinite( input_->points[ idx2 ].x ) || !pcl_isfinite( input_->points[ idx2 ].y ) || !pcl_isfinite( input_->points[ idx2 ].z ) )
				{
					pt2_is_nan = true; 
				}

				if( pt1_is_nan && pt2_is_nan )
				{
					//std::cout << "Pt1 & Pt2 are NaN Neighbors!\n"; 
					return true; 
				}
				else
				{
					return false; 
				}
			}

		protected:
			PointCloudNConstPtr normals_;
			PointCloudLPtr labels_;

			boost::shared_ptr<std::vector<bool> > exclude_labels_;
	};
}
#endif // PCL_SEGMENTATION_NAN_CLUSTER_COMPARATOR_H_
