/**
 * @function SQ_fitter.cpp
 */
#pragma once

#include "SQ_fitter.h"

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
bool SQ_fitter<PointT>::SQFitting( const pcl::PointCloud<PointT> &_cloud ) {

    double ds = (_smax - _smin) / _N;
    double error_i; double error_i_1;
    double s_i;
    bool fitted;

    error_i_1 = initialize( _cloud );

    fitted = false;
    for( int i = 1; i <= _N; ++i ) {
	s_i = _smax - (i-1)*ds;

	pcl::PointCloud<PointT> cloud_i;
	cloud_i = downsample( _cloud, ds );
	error_i = fitting( cloud_i );

	if( fabs( error_i - error_i_1 ) < _thresh ) {
	    fitted = true;
	    break;
	}
    }
    
    return fitted;
}

/**
 * @function initialization
 * @brief Set initial approx. values for parameters
 */    
template<typename PointT>
double SQ_fitter<PointT>::initialize( const pcl::PointCloud<PointT> &_cloud ) {

    // Find the SVD decomposition of the pointcloud
    // Find the pointcloud axis along these 

}

/**
 * @function downsample
 */
template<typename PointT>
pcl::PointCloud<PointT> SQ_fitter<PointT>::downsample( const pcl::PointCloud<PointT> _cloud,
						       double _voxelSize ) {

}

/**
 * @function fitting
 */
template<typename PointT>
double SQ_fitter<PointT>::fitting( const pcl::PointCloud<PointT> &_cloud ) {

}


