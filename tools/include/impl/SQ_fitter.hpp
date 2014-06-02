/**
 * @function SQ_fitter.cpp
 */
#pragma once

#include "SQ_fitter.h"

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>

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

  // Find the bounding box for an initial ellipsoid shape approximation
    pcl::MomentOfInertiaEstimation<PointT> estimator;
    estimator.setInputCloud( _cloud );
    estimator.compute();

    PointT minPt_OBB;
    PointT maxPt_OBB;
    PointT center_OBB;
    Eigen::Matrix3f rotMat_OBB;
    Eigen::Vector3f axisX0, axisY0, axisZ0;
    double a0, b0, c0;

    estimator.getOBB( minPt_OBB, maxPt_OBB, 
		      center_OBB, rotMat_OBB );
    
    estimator.getEigenVectors( axisX0, axisY0, axisZ0 );
    a0 = (maxPt_OBB.x - minPt_OBB.x)*0.5;
    b0 = (maxPt_OBB.y - minPt_OBB.y)*0.5;
    c0 = (maxPt_OBB.z - minPt_OBB.z)*0.5;
    std::cout << "Rot mat: \n"<< rotMat_OBB<<std::endl;
    std::cout << "Ev1: "<<axisX0.transpose() << std::endl;
    std::cout << "Ev2: "<<axisY0.transpose() << std::endl;
    std::cout << "Ev3: "<<axisZ0.transpose() << std::endl;

    // [DEBUG] Visualize bounding box
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("3DViewer") );
    viewer->addCoordinateSystem(1.0, "main", 0);
    viewer->addPointCloud( _cloud, "input cloud"  );
    viewer->addCube( Eigen::Vector3f(center_OBB.x, center_OBB.y, center_OBB.z),
		     Eigen::Quaternionf(rotMat_OBB),
		     a0*2, b0*2, c0*2, "OBB");
    

    while( !viewer->wasStopped() ) {
	viewer->spinOnce(100);
	boost::this_thread::sleep( boost::posix_time::microseconds(100000) );
    }

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


