/**
 * @function SQ_fitter.cpp
 */
#pragma once

#include "SQ_fitter.h"

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

/**
 * @function SQ_fitter
 * @brief Constructor
 */
template<typename PointT>
SQ_fitter<PointT>::SQ_fitter() {

}

/**
 * @function ~SQ_fitter
 * @brief Destructor
 */
template<typename PointT>
SQ_fitter<PointT>::~SQ_fitter() {

}

/**
 * @function SQFitting 
 * @brief Fit a given pointcloud _cloud
 */
template<typename PointT>
bool SQ_fitter<PointT>::SQFitting( const PointCloudPtr &_cloud,
				   const double &_smax,
				   const double &_smin,
				   const int &_N,
				   const double &_thresh ) {
  smax_ = _smax;
  smin_ = _smin;
  N_ = _N;
  thresh_ = _thresh;
  
  double ds = (smax_ - smin_) / N_;
  double error_i; double error_i_1;
  double s_i;
  bool fitted;
  
  error_i_1 = initialize( _cloud );
  
  fitted = false;
  std::cout << "Initial pointcloud size: "<< _cloud->points.size() << std::endl;
  for( int i = 1; i <= N_; ++i ) {
    s_i = _smax - (i-1)*ds;
    
    PointCloudPtr cloud_i( new pcl::PointCloud<PointT>() );
    cloud_i = downsample( _cloud, s_i );
    std::cout << "Iteration "<<i<<" size of cloud: "<< cloud_i->points.size()<< " and size of voxel: "<< s_i << std::endl;
    error_i = fitting( cloud_i );
    
    if( fabs( error_i - error_i_1 ) < thresh_ ) {
      fitted = true;
      //break;
    }
  }
  
  return fitted;
}

/**
 * @function initialization
 * @brief Set initial approx. values for parameters
 */    
template<typename PointT>
double SQ_fitter<PointT>::initialize( const PointCloudPtr &_cloud ) {

    // Find the SVD decomposition of the pointcloud
    // Find the pointcloud axis along these 

  return 0;
}

/**
 * @function downsample
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr SQ_fitter<PointT>::downsample( const PointCloudPtr &_cloud,
							    double _voxelSize ) {
  
  PointCloudPtr downsampled( new pcl::PointCloud<PointT>() );
  
  // Create the filtering object
  pcl::VoxelGrid< PointT > downsampler;
  downsampler.setInputCloud( _cloud );
  downsampler.setLeafSize( _voxelSize, _voxelSize, _voxelSize );
  downsampler.filter( *downsampled );
  
  return downsampled;
}

/**
 * @function fitting
 */
template<typename PointT>
double SQ_fitter<PointT>::fitting( const PointCloudPtr &_cloud ) {
  
  return 0;
}


