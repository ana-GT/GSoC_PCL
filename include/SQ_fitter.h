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
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  void setInputCloud( const PointCloudPtr &_cloud );
  bool fit();

  bool minimize( const PointCloudPtr &_cloud, 
		 const SQ_parameters &_in,
		 SQ_parameters &_out );

  void printResults();
  void visualize();

 private:
  SQ_parameters par_in_;
  SQ_parameters par_out_;

  PointCloudPtr cloud_;

};


#include "impl/SQ_fitter.hpp"
