/**
 * @file SQ_fitter.h
 */
#pragma once

#include <pcl/pcl_base.h>
#include "SQ_params.h"

/**
 * @class SQ_fitter
 */
template <typename PointT>
class SQ_fitter {
  
 public:

  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

  SQ_fitter();
  ~SQ_fitter();
  bool SQFitting( const PointCloudPtr &_cloud,
		  const double &_smax = 0.05,
		  const double &_smin = 0.01,
		  const int &_N = 5,
		  const double &_thresh = 0.1 );    
  double initialize( const PointCloudPtr &_cloud );
  typename pcl::PointCloud<PointT>::Ptr downsample( const PointCloudPtr &_cloud,
			    double _voxelSize );
  double fitting( const PointCloudPtr &_cloud,
		  const SQ_params &_par_in,
		  SQ_params &_par_out);
  double error( const PointCloudPtr &_cloud,
		const SQ_params &_params );

 private:

    bool fitting();

    double smax_; /**< Maximum voxel size */
    double smin_; /**< Minimum voxel size */
    int N_; /**< Number of scales */
    double thresh_; /**< Error threshold */
    
    SQ_params par0_;
};


#include "impl/SQ_fitter.hpp"
