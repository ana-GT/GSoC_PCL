/**
 * @function SQ_fitter.hpp
 */
#pragma once

#include "SQ_fitter.h"
#include "SQ_sampler.h"
#include "SQ_utils.h"
#include "minimizer.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>

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

    // Store parameters
    smax_ = _smax;
    smin_ = _smin;
    N_ = _N;
    thresh_ = _thresh;
    cloud_in_ = _cloud;
    
    double ds;
    double error_i; double error_i_1;
    double s_i;
    bool fitted;
    SQ_params par_i, par_i_1;
  
    // 1. Initialize a ellipsoid with bounding box dimensions
    ds = (smax_ - smin_) / (double) N_;
    error_i = initialize( cloud_in_, par_in_ );
    par_i = par_in_;

    std::cout << "\t [DEBUG] Iteration ["<<0<<"] Cloud size: "<< cloud_in_->points.size()<< ". Error: "<< error_i << std::endl;

    std::cout << "\t Initialization parameters:"<< std::endl;
    printParamsInfo( par_in_ );


    // 2. Iterate through downsampling levels
    fitted = false;    
    for( int i = 1; i <= N_; ++i ) {
	
	error_i_1 = error_i;
	par_i_1 = par_i;
	s_i = _smax - (i-1)*ds;
	
	PointCloudPtr cloud_i( new pcl::PointCloud<PointT>() );
	cloud_i = downsample( cloud_in_, s_i );
	error_i = fitting( cloud_i, 
			   par_i_1, par_i );
	
	std::cout << "\t [DEBUG] Iteration ["<<i<<"] Cloud size: "<< cloud_i->points.size()<< 
	    ". Voxel size: "<< s_i <<". Error: "<< error_i << std::endl;

	// 3. If error between levels is below threshold, stop
	if( fabs( error_i - error_i_1 ) < thresh_ ) {
	    std::cout << "\t [DEBUG-GOOD] Errors diff below threshold. Stop: "<< thresh_ << std::endl;
	    par_out_ = par_i;
	    fitted = true;
	    break;
	}
    }
    
    return fitted;
}

/**
 * @function getBoundingBox
 * @brief Set segmented cloud to be fitted
 */
template<typename PointT>
void SQ_fitter<PointT>::getBoundingBox(const PointCloudPtr &_cloud,
				       double _dim[3],
				       double _trans[3],
				       double _rot[3] ) {

  // 1. Compute the bounding box center
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid( *_cloud, centroid );
  _trans[0] = centroid(0);
  _trans[1] = centroid(1); 
  _trans[2] = centroid(2);

  // 2. Compute main axis orientations
  pcl::PCA<PointT> pca;
  pca.setInputCloud( _cloud );
  Eigen::Vector3f eigVal = pca.getEigenValues();
  Eigen::Matrix3f eigVec = pca.getEigenVectors();
  // Make sure 3 vectors are normal w.r.t. each other
  Eigen::Vector3f temp;
  eigVec.col(2) = eigVec.col(0); // Z
  Eigen::Vector3f v3 = (eigVec.col(1)).cross( eigVec.col(2) );
  eigVec.col(0) = v3;
  Eigen::Vector3f rpy = eigVec.eulerAngles(2,1,0);
 
  _rot[0] = (double)rpy(2);
  _rot[1] = (double)rpy(1);
  _rot[2] = (double)rpy(0);

  // Transform _cloud
  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  transf.block(0,3,3,1) << (float)centroid(0), (float)centroid(1), (float)centroid(2);
  transf.block(0,0,3,3) = eigVec;

  Eigen::Matrix4f tinv; tinv = transf.inverse();
  PointCloudPtr cloud_temp( new pcl::PointCloud<PointT>() );
  pcl::transformPointCloud( *_cloud, *cloud_temp, tinv );

  // Get maximum and minimum
  PointT minPt; PointT maxPt;
  pcl::getMinMax3D( *cloud_temp, minPt, maxPt );
  
  _dim[0] = ( maxPt.x - minPt.x ) / 2.0;
  _dim[1] = ( maxPt.y - minPt.y ) / 2.0;
  _dim[2] = ( maxPt.z - minPt.z ) / 2.0;

}


/**
 * @function initialize
 * @brief Set initial approx. values for parameters
 */    
template<typename PointT>
double SQ_fitter<PointT>::initialize( const PointCloudPtr &_cloud,
				      SQ_params &_par_out ) {

  double dim[3]; double trans[3]; double rot[3];
  getBoundingBox( _cloud, dim, trans, rot );
		  
  _par_out.px = trans[0];
  _par_out.py = trans[1]; 
  _par_out.pz = trans[2];
 
  _par_out.ra = rot[0];
  _par_out.pa = rot[1];
  _par_out.ya = rot[2];
  
  _par_out.a = dim[0];
  _par_out.b = dim[1];
  _par_out.c = dim[2];

  _par_out.e1 = 0.5; _par_out.e2 = 0.5;
    
  return this->error( _par_out );
}

/**
 * @function downsample
 * @brief Reduce the number of points in the cloud by using voxelization
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr SQ_fitter<PointT>::downsample( const PointCloudPtr &_cloud,
							    double _voxelSize ) {
  
  PointCloudPtr downsampled( new pcl::PointCloud<PointT>() );
  
  // Create the filtering object
  pcl::VoxelGrid< PointT > downsampler;
  // Set input cloud
  downsampler.setInputCloud( _cloud );
  // Set size of voxel
  downsampler.setLeafSize( _voxelSize, _voxelSize, _voxelSize );
  // Downsample
  downsampler.filter( *downsampled );
  
  return downsampled;
}

/**
 * @function fitting
 * @brief Fit a superquadric to _cloud using as initial parameters _par_in
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
    return this->error( _par_out );
}

/**
 * @function error
 * @brief Calculate the error from the _cloud w.r.t. the SQ defined by _params
 */
template<typename PointT>
double SQ_fitter<PointT>::error( const SQ_params &_par ) {

    double sum; double err;
    
    double x,y,z;
    pcl::PointXYZ p;

    sum = 0;
    for( int i = 0; i < cloud_in_->points.size(); ++i ) {
	p = cloud_in_->points[i];
	x = p.x; y = p.y; z = p.z;
	err = error_MATLAB( params2Vec(_par), x, y, z );
	sum += err;
    }

    return sum / (double) cloud_in_->points.size();
}

/**
 * @function visualizeFit
 * @brief Debug function to evaluate how good the fitting is
 */
template<typename PointT>
void SQ_fitter<PointT>::visualizeFit() {

    std::cout << "Calling visualize fit"<< std::endl;

    // [DEBUG] Visualize 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("3DViewer") );
    viewer->addCoordinateSystem(1.0, "main", 0);

    // [DEBUG] Visualize input pointcloud (GREEN)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col(cloud_in_, 0,255,0);
    viewer->addPointCloud( cloud_in_, col, "input cloud"  );

    // [DEBUG] Visualize bounding box
    Eigen::Matrix3f rot;
    rot = Eigen::AngleAxisf( par_in_.ya, Eigen::Vector3f::UnitZ())*
	Eigen::AngleAxisf( par_in_.pa, Eigen::Vector3f::UnitY())*
	Eigen::AngleAxisf( par_in_.ra, Eigen::Vector3f::UnitX());
    viewer->addCube( Eigen::Vector3f( (float)par_in_.px,(float)par_in_.py,(float)par_in_.pz),
		     Eigen::Quaternionf(rot),
		     par_in_.a*2, par_in_.b*2, par_in_.c*2, "OBB");
    
    // [DEBUG] Visualize initial ellipsoid (RED)
    visualizeSQ( viewer, 
		 par_in_, std::string("se0"), 255, 0, 0 );

    // [DEBUG] Visualize final fitted ellipsoid (BLUE)
    visualizeSQ( viewer, 
		 par_out_, std::string("sef"), 0, 0, 255 );

    while( !viewer->wasStopped() ) {
	viewer->spinOnce(100);
	boost::this_thread::sleep( boost::posix_time::microseconds(100000) );
    }

    std::cout << "\t [DEBUG] Initial parameters"<< std::endl;
    printParamsInfo( par_in_ );

    std::cout << "\t [DEBUG] Final parameters"<< std::endl;
    printParamsInfo( par_out_ );

}
