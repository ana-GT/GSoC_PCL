/**
 * @file SQ_fitter.h
 */
#pragma once

#include "SQ_parameters.h"
#include <pcl/io/pcd_io.h>

/**
 * @class SQ_fitter
 */
template <typename PointT>
class SQ_fitter {

 public:

  SQ_fitter();
  ~SQ_fitter();

  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

  void setInputCloud( const PointCloudPtr &_cloud );
  void getBoundingBox(const PointCloudPtr &_cloud,
		      double _dim[3],
		      double _trans[3],
		      double _rot[3] );
  bool fit( const double &_smax = 0.05,
	    const double &_smin = 0.01,
	    const int &_N = 5,
	    const double &_thresh = 0.1 );

  void downsample( const PointCloudPtr &_cloud,
		   const double &_voxelSize,
		   PointCloudPtr &_cloud_downsampled );

  bool minimize( const PointCloudPtr &_cloud, 
		 const SQ_parameters &_in,
		 SQ_parameters &_out,
		 double &_error );

  double error_metric( SQ_parameters _par,
		       const PointCloudPtr &_cloud );

  void printResults();
  void visualize();

 private:
  SQ_parameters par_in_;
  SQ_parameters par_out_;

  PointCloudPtr cloud_;

  double smax_; /**< Maximum voxel size */
  double smin_; /**< Minimum voxel size */
  int N_; /**< Number of scales */
  double thresh_; /**< Error threshold */

};


#include "impl/SQ_fitter.hpp"
