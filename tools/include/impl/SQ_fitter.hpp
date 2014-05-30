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

}

/**
 * @function initialization
 * @brief Set initial approx. values for parameters
 */    
template<typename PointT>
bool SQ_fitter<PointT>::initialization() {

}

/**
 * @function fitting
 * @brief Apply LMS at a scale level
 */
template<typename PointT>
bool SQ_fitter<PointT>::fitting() {

}
