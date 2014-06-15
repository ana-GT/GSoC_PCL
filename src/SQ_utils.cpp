/**
 * @file SQ_utils.cpp
 */
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include "SQ_utils.h"
#include <iostream>

/**
 * @function printParamsInfo
 * @brief Print in a human-friendly form the parameters info
 */
void printParamsInfo( const SQ_parameters &_par ) {

    std::cout << "\t ** Params: **"<< std::endl;
    std::cout << "\t Axis dimensions: "<< _par.dim[0] << ", "<< _par.dim[1]<<", "<< _par.dim[2] << std::endl;
    std::cout << "\t Epsilons: "<< _par.e[0] << ", "<< _par.e[1]<< std::endl;
    std::cout << "\t Trans: "<< _par.trans[0] << ", "<< _par.trans[1]<<", "<< _par.trans[2] << std::endl;
   std::cout << "\t Rot: "<< _par.rot[0] << ", "<< _par.rot[1]<<", "<< _par.rot[2] << std::endl;
}


/***
 * @function visualizeSQ
 * @brief Add sampled Super Quadric to viewer (red color by default)
 */
void visualizeSQ(  boost::shared_ptr<pcl::visualization::PCLVisualizer> &_viewer,
		   const SQ_parameters &_par,
		   std::string _name,
		   int _r, int _g, int _b ) {

    // Create sample pointcloud of SQ expressed by _par
  pcl::PointCloud<pcl::PointXYZ>::Ptr sq_pcd( new pcl::PointCloud<pcl::PointXYZ>() );
  sq_pcd = sampleSQ_naive( _par );
  
    // Add it 
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color( sq_pcd, _r, _g, _b );
  _viewer->addPointCloud<pcl::PointXYZ> ( sq_pcd, color, _name );	
  
}


/**
 * @function sampleSQ_naive
 * @brief Sample n \in [-PI/2, PI/2] and w \in [-PI,PI>
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSQ_naive( const SQ_parameters &_par ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ> cloud_raw;
  
  double cn, sn, sw, cw;
  double n, w;
  int num_n, num_w; double dn, dw;
  
  dn = 5.0*M_PI / 180.0;
  dw = 5.0*M_PI / 180.0;
  num_n = (int)( M_PI / dn );
  num_w = (int)( 2*M_PI / dw );
  
  double a, b, c, e1, e2;
  a = _par.dim[0]; b = _par.dim[1]; c = _par.dim[2]; 
  e1 = _par.e[0]; e2 = _par.e[1];
  
  n = -M_PI / 2.0;
  for( int i = 0; i < num_n; ++i ) {
    n += dn;
    cn = cos(n); sn = sin(n);
    w = -M_PI;
    
    for( int j = 0; j < num_w; ++j ) {
      w += dw;
      cw = cos(w); sw = sin(w);
      pcl::PointXYZ p;
      p.x = a*pow( fabs(cn), e1 )*pow( fabs(cw), e2 );
      p.y = b*pow( fabs(cn), e1 )*pow( fabs(sw), e2 );
      p.z = c*pow( fabs(sn), e1 );
      
      // Assign signs, if needed
      if( cn*cw < 0 ) { p.x = -p.x; }
      if( cn*sw < 0 ) { p.y = -p.y; }
      if( sn < 0 ) { p.z = -p.z; }
      
      // Store
      cloud_raw.points.push_back(p);	    
    }
  }
  
  // Apply transform
  Eigen::Matrix4d transf = Eigen::Matrix4d::Identity();
  transf.block(0,3,3,1) = Eigen::Vector3d( _par.trans[0], _par.trans[1], _par.trans[2] );
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd( _par.rot[2], Eigen::Vector3d::UnitZ() )*
    Eigen::AngleAxisd( _par.rot[1], Eigen::Vector3d::UnitY() )*
    Eigen::AngleAxisd( _par.rot[0], Eigen::Vector3d::UnitX() );
  transf.block(0,0,3,3) = rot;
  pcl::transformPointCloud( cloud_raw,
			    *cloud,
			    transf );
  
  cloud->height = 1;
  cloud->width = cloud->points.size();
  
  return cloud;
}

