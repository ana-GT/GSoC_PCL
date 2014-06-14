/**
 * @file SQ_fitter.hpp
 */

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <ceres/ceres.h> 
#include "SQ_fitter.h"


struct SQBaseEquation {

  SQBaseEquation( double _x, double _y, double _z ) : 
    x_(_x), y_(_y), z_(_z) {}
  template<typename T> bool operator() ( const T* const dim,
					 const T* const e,
					 T* residual ) const {
    

    // F(x) : Base SQ equation
    T F; T fxy; T fz;
    fxy = pow( ceres::abs( T(x_)/dim[0]), T(2.0)/e[1] ) + pow( ceres::abs(T(y_)/dim[1]), T(2.0)/e[1] );
    fz =  pow( ceres::abs( T(z_)/dim[2]), T(2.0)/e[0] );
    F = pow( ceres::abs(fxy), T(e[1]/e[0]) ) + fz;    
    residual[0] = pow(F, e[1]) -T(1.0);
    return true;
    
  }

private:
  double x_;
  double y_;
  double z_;
};



/**
 * @function setInputCloud
 * @brief Set segmented cloud to be fitted
 */
template<typename PointT>
void SQ_fitter<PointT>::setInputCloud( const PointCloudPtr &_cloud ) {
  cloud_ = _cloud;
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
  PointCloudPtr cloud_norm( new pcl::PointCloud<PointT>() );
  Eigen::Matrix4f transf = Eigen::Matrix4f();
  transf.block(0,3,3,1) << (float)centroid(0), (float)centroid(1), (float)centroid(2);
  transf.block(0,0,3,3) = eigVec;
  Eigen::Matrix4f tinv; tinv = transf.inverse();
  pcl::transformPointCloud( *_cloud, *cloud_norm, tinv );

  // Get maximum and minimum
  PointT minPt; PointT maxPt;
  pcl::getMinMax3D( *cloud_norm, minPt, maxPt );
  
  _dim[0] = ( maxPt.x - minPt.x ) / 2.0;
  _dim[1] = ( maxPt.y - minPt.y ) / 2.0;
  _dim[2] = ( maxPt.z - minPt.z ) / 2.0;

}

/**
 * @function fit
 * @brief Fit using Levenberg-Marquadt with box constraints
 */
template<typename PointT>
bool SQ_fitter<PointT>::fit() {

  // Initialize par_in_ with bounding box values
  getBoundingBox( cloud_, 
		  par_in_.dim,
		  par_in_.trans,
		  par_in_.rot );
  // Set e1 and e2 to middle value in range
  par_in_.e[0] = 1.0; par_in_.e[1] = 1.0;

  std::cout << "Dim:"<< par_in_.dim[0]<<", "<<par_in_.dim[2]<<", "<<par_in_.dim[3]<<std::endl;

  // Run loop


  return true;
}

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

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve( options, &problem, &summary );
}


template<typename PointT>
void SQ_fitter<PointT>::printResults() {
  
  // Visualize final results
  //std::cout << summary.BriefReport() << std::endl;
}

template<typename PointT>
void SQ_fitter<PointT>::visualize() {

  // Create viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("SQ_fitter Viewer") );
  viewer->addCoordinateSystem(1.0, 0);
  
  // Visualize input pointcloud (GREEN)
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col(cloud_, 0,255,0);
  viewer->addPointCloud( cloud_, col, "Input cloud" );

  Eigen::Matrix3f rot; rot = Eigen::AngleAxisf( (float)par_in_.rot[2], Eigen::Vector3f::UnitZ() )*
			Eigen::AngleAxisf( (float)par_in_.rot[1], Eigen::Vector3f::UnitY() )*
			Eigen::AngleAxisf( (float)par_in_.rot[0], Eigen::Vector3f::UnitX() );
  viewer->addCube( Eigen::Vector3f( (float)par_in_.trans[0],
				    (float)par_in_.trans[1],
				    (float)par_in_.trans[2]),
		   Eigen::Quaternionf(rot),
		   par_in_.dim[0]*2, par_in_.dim[1]*2, par_in_.dim[2]*2, 
		   "OBB");

  // Visualize output fitted pointcloud (RED)

  // Visualize initial guess (BLUE)
  /*
  SQ_sampler sqs;
  SQ_params par;
  par.a = dim[0]; par.b = dim[1]; par.c = dim[2];
  par.e1 = e[0]; par.e2 = e[1];
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudF( new pcl::PointCloud<pcl::PointXYZ>() );
  cloudF = sqs.sampleSQ_naive( par );

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colF(cloudF, 255,0,0);
  viewer->addPointCloud( cloudF, colF, "Final cloud"  );
  */

  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000) );
  }

  
}
