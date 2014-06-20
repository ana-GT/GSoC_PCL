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

  /**< Main fitting function */
  bool SQFitting( const PointCloudPtr &_cloud,
		  const double &_smax = 0.05,
		  const double &_smin = 0.01,
		  const int &_N = 5,
		  const double &_thresh = 0.1 );   
   

  void getBoundingBox( const PointCloudPtr &_cloud,
		       double _dim[3],
		       double _trans[3],
		       double _rot[3] );

  /**< Getters */
  SQ_params get_init_params() { return par_in_; }
  SQ_params get_fit_params() { return par_out_; }
  
  /**< Debug Visualization of results */
  void visualizeFit();

 private:

  double initialize( const PointCloudPtr &_cloud, 
		     SQ_params &_par_out );
  typename pcl::PointCloud<PointT>::Ptr downsample( const PointCloudPtr &_cloud,
						    double _voxelSize );
  double fitting( const PointCloudPtr &_cloud,
		  const SQ_params &_par_in,
		  SQ_params &_par_out);
  double error( const SQ_params &_params );
  
  
  
  double smax_; /**< Maximum voxel size */
  double smin_; /**< Minimum voxel size */
  int N_; /**< Number of scales */
  double thresh_; /**< Error threshold */
  
  SQ_params par_in_;
  SQ_params par_out_;
  PointCloudPtr cloud_in_;
};


#include "impl/SQ_fitter.hpp"
