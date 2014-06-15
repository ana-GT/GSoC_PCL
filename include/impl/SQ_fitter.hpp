/**
 * @file SQ_fitter.hpp
 * @brief Implementation
 */

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <ceres/ceres.h> 

#include "SQ_fitter.h"
#include "SQ_utils.h"
#include "SQ_structs.h"


/**
 * @function SQ_fitter
 * @brief Constructor. Create initial pointers for cloud and its normalized version
 */
template<typename PointT>
SQ_fitter<PointT>::SQ_fitter() :
  cloud_( new pcl::PointCloud<PointT>() ),
  cloud_norm_( new pcl::PointCloud<PointT>() ) {

}

/**
 * @function ~SQ_fitter
 * @brief Destructor
 */
template<typename PointT>
SQ_fitter<PointT>::~SQ_fitter() {

}


/**
 * @function setInputCloud
 * @brief Set segmented cloud to be fitted
 */
template<typename PointT>
void SQ_fitter<PointT>::setInputCloud( const PointCloudPtr &_cloud ) {
  cloud_ = _cloud;
}

/**
 * @function fit
 * @brief Fit using Levenberg-Marquadt with box constraints
 */
template<typename PointT>
bool SQ_fitter<PointT>::fit( const double &_smax,
			     const double &_smin,
			     const int &_N,
			     const double &_thresh ) {

  // 0. Store parameters
  smax_ = _smax;
  smin_ = _smin;
  N_ = _N;
  thresh_ = _thresh;

  double ds; double error_i; double error_i_1;
  double s_i; bool fitted; 
  SQ_parameters par_i, par_i_1;

  ds = (smax_ - smin_) / (double) N_;

  // 1. Initialize par_in_ with bounding box values
  getBoundingBox( cloud_, 
		  par_in_.dim,
		  par_in_.trans,
		  par_in_.rot,
		  cloud_norm_ );
  // 1.1. Set e1 and e2 to middle value in range
  par_in_.e[0] = 1.0; par_in_.e[1] = 1.0;
 
  // Initial paras
  std::cout << "Very first parameters"<<std::endl;
  printParamsInfo(par_in_);
  // Run loop
  par_i = par_in_;

  fitted = false;
  for( int i = 1; i <= N_; ++i ) {

    s_i = smax_ -(i-1)*ds;
    std::cout << "Iteration "<<i<< std::endl;
    std::cout << "Voxel size: "<< s_i << std::endl;
    par_i_1 = par_i;

    PointCloudPtr cloud_i( new pcl::PointCloud<PointT>() );
    downsample( cloud_norm_,
		s_i,
		cloud_i );
    std::cout << "Size of pointcloud evaluated: "<< cloud_i->points.size() << std::endl;
    minimize( cloud_i,
	      par_i_1,
	      par_i );
  
    // Check
    std::cout << "Output at iteration:"<< i << std::endl;
    printParamsInfo( par_i );

  }
 
  par_out_ = par_i;

  return true;
}


/**
 * @function getBoundingBox
 * @brief Set segmented cloud to be fitted
 */
template<typename PointT>
void SQ_fitter<PointT>::getBoundingBox(const PointCloudPtr &_cloud,
				       double _dim[3],
				       double _trans[3],
				       double _rot[3],
				       PointCloudPtr &_cloud_norm ) {

  // 1. Compute the bounding box center
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid( *_cloud, centroid );
  _trans[0] = centroid(0); _trans[1] = centroid(1); _trans[2] = centroid(2);

  // 2. Compute main axis orientations
  pcl::PCA<PointT> pca;
  pca.setInputCloud( _cloud );
  Eigen::Vector3f eigVal = pca.getEigenValues();
  Eigen::Matrix3f eigVec = pca.getEigenVectors();
  // Make sure 3 vectors are normal w.r.t. each other
  Eigen::Vector3f v3 = (eigVec.col(0)).cross( eigVec.col(1) );
  eigVec.col(2) = v3;
  Eigen::Vector3f rpy = eigVec.eulerAngles(2,1,0);
 
  _rot[0] = (double)rpy(2);
  _rot[1] = (double)rpy(1);
  _rot[2] = (double)rpy(0);

  // Transform _cloud
  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  transf.block(0,3,3,1) << (float)centroid(0), (float)centroid(1), (float)centroid(2);
  transf.block(0,0,3,3) = eigVec;

  Eigen::Matrix4f tinv; tinv = transf.inverse();
  pcl::transformPointCloud( *_cloud, *_cloud_norm, tinv );

  // Get maximum and minimum
  PointT minPt; PointT maxPt;
  pcl::getMinMax3D( *_cloud_norm, minPt, maxPt );
  
  _dim[0] = ( maxPt.x - minPt.x ) / 2.0;
  _dim[1] = ( maxPt.y - minPt.y ) / 2.0;
  _dim[2] = ( maxPt.z - minPt.z ) / 2.0;

}

/**
 * @function downsample
 * @brief Fit using Levenberg-Marquadt with box constraints
 */
template<typename PointT>
void SQ_fitter<PointT>::downsample( const PointCloudPtr &_cloud,
				    const double &_voxelSize,
				    PointCloudPtr &_cloud_downsampled ) {
    
  // Create the filtering object
  pcl::VoxelGrid< PointT > downsampler;
  // Set input cloud
  downsampler.setInputCloud( _cloud );
  // Set size of voxel
  downsampler.setLeafSize( _voxelSize, _voxelSize, _voxelSize );
  // Downsample
  downsampler.filter( *_cloud_downsampled );
  
}


/**
 * @function minimize
 * @brief Apply bounded Levenberg-Marquadt with _in initial parameters
 * @output _out
 */
template<typename PointT>
bool SQ_fitter<PointT>::minimize( const PointCloudPtr &_cloud, 
				  const SQ_parameters &_in,
				  SQ_parameters &_out ) {

  // Parameters initially _in:
  _out = _in; 
  
  // Create problem 
  ceres::Problem problem;
  // Add residual blocks
  typename pcl::PointCloud<PointT>::iterator it;
  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {
    problem.AddResidualBlock( new ceres::AutoDiffCostFunction<SQBaseEquation, 1, 3, 2>(new SQBaseEquation((*it).x, 
													  (*it).y, 
													  (*it).z)),
			      NULL,
			      _out.dim, _out.e );
  }
  
  
  // Set limits for principal axis of Super Quadric
  for( int i = 0; i < 3; ++i ) {
    problem.SetParameterLowerBound( _out.dim, i, 0.01 );
    problem.SetParameterUpperBound( _out.dim, i, 0.8 );
  }
  // Set limits for coefficients e1 and e2
  for( int i = 0; i < 2; ++i ) {
    problem.SetParameterLowerBound( _out.e, i, 0.1 );
    problem.SetParameterUpperBound( _out.e, i, 1.9 );  
  }


  // Set options
  ceres::Solver::Options options;
  options.max_num_iterations = 50;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_type = ceres::TRUST_REGION;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve( options, &problem, &summary );
}

/**
 * @function printResults
 * @brief Print results of optimization (initial vs final fitted parameters)
 */
template<typename PointT>
void SQ_fitter<PointT>::printResults() {
  
  // 1. Print initial parameters information
  std::cout << "\t [Optimization Results] Initial parameters: "<< std::endl;
  printParamsInfo( par_in_ );
  // 2. Print final parameters information
  std::cout << "\t [Optimization results] Final parameters: "<< std::endl;
  printParamsInfo( par_out_ );

}

/**
 * @function visualize
 * @brief Visualize pointcloud and final fit
 */
template<typename PointT>
void SQ_fitter<PointT>::visualize() {

  // 1. Create viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("SQ_fitter Viewer") );
  viewer->addCoordinateSystem(1.0, 0);
  
  // 2. Visualize input pointcloud (GREEN)
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col(cloud_norm_, 0,255,0);
  viewer->addPointCloud( cloud_norm_, col, "Input cloud" );

  Eigen::Matrix3f rot; rot = Eigen::AngleAxisf( (float)par_in_.rot[2], Eigen::Vector3f::UnitZ() )*
			Eigen::AngleAxisf( (float)par_in_.rot[1], Eigen::Vector3f::UnitY() )*
			Eigen::AngleAxisf( (float)par_in_.rot[0], Eigen::Vector3f::UnitX() );
  viewer->addCube( Eigen::Vector3f( (float)par_in_.trans[0],
				    (float)par_in_.trans[1],
				    (float)par_in_.trans[2]),
		   Eigen::Quaternionf(rot),
		   par_in_.dim[0]*2, par_in_.dim[1]*2, par_in_.dim[2]*2, 
		   "OBB");
  
  // 3. Visualize output fitted pointcloud (RED)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out( new pcl::PointCloud<pcl::PointXYZ>() );
  cloud_out = sampleSQ_naive( par_out_ );

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col_out(cloud_out, 255,0,0);
  viewer->addPointCloud( cloud_out, col_out, "Cloud fitted"  );

  // Visualize initial guess (BLUE)
  /*
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in( new pcl::PointCloud<pcl::PointXYZ>() );
  cloud_in = sampleSQ_naive( par_in_ );

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col_in(cloud_in, 0,0,255);
  viewer->addPointCloud( cloud_in, col_in, "Cloud initial"  );
  */

  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000) );
  }

  
}
