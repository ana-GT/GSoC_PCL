/**
 * @function SQ_fitter.cpp
 */
#pragma once

#include "SQ_fitter.h"
#include "SQ_sampler.h"
#include "SQ_utils.h"
#include "minimizer.h"
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
  SQ_params par_i;
  
  error_i = initialize( _cloud );
  
  fitted = false;
  std::cout << "Initial pointcloud size: "<< _cloud->points.size() << std::endl;
  for( int i = 1; i <= N_; ++i ) {
      
      error_i_1 = error_i;
      s_i = _smax - (i-1)*ds;
      
      PointCloudPtr cloud_i( new pcl::PointCloud<PointT>() );
      cloud_i = downsample( _cloud, s_i );
      std::cout << "Iteration "<<i<<" size of cloud: "<< cloud_i->points.size()<< " and size of voxel: "<< s_i << std::endl;
      error_i = fitting( cloud_i, 
			 par0_, par_i );
      
      if( fabs( error_i - error_i_1 ) < thresh_ ) {
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

    // Same info as in rotMat_OBB (or it should be)
    //estimator.getEigenVectors( axisX0, axisY0, axisZ0 );
    par0_.a = (maxPt_OBB.x - minPt_OBB.x)*0.5;
    par0_.b = (maxPt_OBB.y - minPt_OBB.y)*0.5;
    par0_.c = (maxPt_OBB.z - minPt_OBB.z)*0.5;
    par0_.e1 = 0.5; par0_.e2 = 0.5;

    Eigen::Isometry3d Tf = Eigen::Isometry3d::Identity();
    Tf.translation() = Eigen::Vector3d(center_OBB.x, center_OBB.y, center_OBB.z );
    Tf.linear() << (double)rotMat_OBB(0,0), (double)rotMat_OBB(0,1), (double)rotMat_OBB(0,2), 
	(double)rotMat_OBB(1,0), (double)rotMat_OBB(1,1), (double)rotMat_OBB(1,2), 
	(double)rotMat_OBB(2,0), (double)rotMat_OBB(2,1), (double)rotMat_OBB(2,2);
   
    transf2Params( Tf, par0_ );

    
    // [DEBUG] Visualize 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("3DViewer") );
    viewer->addCoordinateSystem(1.0, "main", 0);

    // [DEBUG] Visualize input pointcloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col(_cloud, 0,255,0);
    viewer->addPointCloud( _cloud, col, "input cloud"  );

    // [DEBUG] Visualize bounding box
    viewer->addCube( Eigen::Vector3f(center_OBB.x, center_OBB.y, center_OBB.z),
		     Eigen::Quaternionf(rotMat_OBB),
		     par0_.a*2, par0_.b*2, par0_.c*2, "OBB");
    
    // [DEBUG] Visualize initial ellipsoid
    SQ_sampler sqs;
    pcl::PointCloud<pcl::PointXYZ>::Ptr se0( new pcl::PointCloud<pcl::PointXYZ>() );
    se0 = sqs.sampleSQ_naive( par0_ );
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> se0_col(se0, 255,0,0);
    viewer->addPointCloud( se0, se0_col, "Ellipsoid 0" );


    while( !viewer->wasStopped() ) {
	viewer->spinOnce(100);
	boost::this_thread::sleep( boost::posix_time::microseconds(100000) );
    }

    printParamsInfo( par0_ );
    return this->error( _cloud, par0_ );
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
double SQ_fitter<PointT>::fitting( const PointCloudPtr &_cloud,
				   const SQ_params &_par_in,
				   SQ_params &_par_out ) {
  
    minimizer mini;
    mini.loadPoints( _cloud );
    mini.minimize( _par_in,
		   _par_out );

    // Return error
    return this->error( _cloud, _par_out );
}

/**
 * @function error
 * @brief Calculate the error from the _cloud w.r.t. the SQ defined by _params
 */
template<typename PointT>
double SQ_fitter<PointT>::error( const PointCloudPtr &_cloud,
				 const SQ_params &_par ) {

    double sum; double err;
    
    double x,y,z;
    pcl::PointXYZ p;

    sum = 0;
    for( int i = 0; i < _cloud->points.size(); ++i ) {
	p = _cloud->points[i];
	x = p.x; y = p.y; z = p.z;
	err = error_MATLAB( params2Vec(_par), x, y, z );
	sum += err;
    }

    return sum;
}

