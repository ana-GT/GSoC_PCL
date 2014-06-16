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

/**
 * @function dTheta
 */
double dTheta( double K,
	       double e,
	       double a1, double a2,
	       double t ) {

  double num = (cos(t)*cos(t)*sin(t)*sin(t));
  double den1 = a1*a1*pow( fabs(cos(t)),2*e)*pow( fabs(sin(t)),4);
  double den2 = a2*a2*pow( fabs(sin(t)),2*e)*pow( fabs(cos(t)),4);
  return (K/e)*sqrt( num/(den1+den2) );
}


/**
 * @function dTheta
 */
double dTheta_0( double K,
		 double e,
		 double a1, double a2,
		 double t ) {
  double factor = K /a2 - pow(t, e);
  double po = pow( fabs(factor), 1.0/e );
  double m = fabs(po - t);
  return m;
}

/**
 * @file sampleSQ_uniform
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSQ_uniform( const double &_a1, 
						      const double &_a2,
						      const double &_a3,
						      const double &_e1,
						      const double &_e2,
						      const int &_N ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );

  pcl::PointCloud<pcl::PointXYZ>::Ptr s1( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr s2( new pcl::PointCloud<pcl::PointXYZ>() );
  s1 = sampleSE_uniform( 1, _a3, _e1, _N );
  s2 = sampleSE_uniform( _a1, _a2, _e2, _N );

  pcl::PointCloud<pcl::PointXYZ>::iterator it1;
  pcl::PointCloud<pcl::PointXYZ>::iterator it2;

  for( it1 = s1->begin(); it1 != s1->end(); ++it1 ) {
    for( it2 = s2->begin(); it2 != s2->end(); ++it2 ) {
      pcl::PointXYZ p;
      p.x = (*it1).x*(*it2).x;
      p.y = (*it1).x*(*it2).y;
      p.z = (*it1).y;
      cloud->points.push_back(p);
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

/**
 * @function sampleSE_uniform
 * @brief Sample SuperEllipse with Pilu and Fischer method
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSE_uniform( const double &_a1, 
						      const  double &_a2,
						      const double &_e,
						      const int &_N ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ> cloud_base;

  double theta;
  double thresh = 0.01;
  int numIter;
  int maxIter = 500;
  double K;

  if( _a1 < _a2 ) { K = 2*M_PI*_a1/(double)_N; } 
  else { K = 2*M_PI*_a2/(double)_N; }
 
  // theta \in [0,thresh]
  theta = 0;
  numIter = 0;
  
  do {
    double dt = dTheta_0(K, _e, _a1, _a2, theta );
    theta += dt;
    std::cout <<"["<<numIter<<"]Theta: "<< std::fixed<<theta<< std::endl;
    if( dt > 0 ) { std::cout << "dt > 0" << std::endl;}
    numIter++;

    if( dt != 0 ) {
      pcl::PointXYZ p;
      p.x = _a1*pow( fabs(cos(theta)), _e );
      p.y = _a2*pow( fabs(sin(theta)), _e );
      p.z = 0;
      cloud_base.push_back(p);
    }
  } while( theta < thresh   && numIter < maxIter );
 
  std::cout << "[0,thresh]Cloud size: "<< cloud_base.points.size()<<std::endl;

  // theta \in [thresh, PI/2 - thresh]
  if( theta < thresh ) { theta = thresh; }
  numIter = 0;
  do {
    theta += dTheta( K, _e, _a1, _a2, theta ); 
    numIter++;

    pcl::PointXYZ p;
    p.x = _a1*pow( fabs(cos(theta)), _e );
    p.y = _a2*pow( fabs(sin(theta)), _e );
    p.z = 0;
    cloud_base.push_back(p);

  } while( theta < M_PI/2.0 - thresh  && numIter < maxIter );

 std::cout << "After big stint: "<< cloud_base.points.size() << std::endl;

 // theta \in [PI/2 - thresh, PI/2]
 double alpha = M_PI/2.0 - theta;
 numIter = 0;
 do {
   alpha -= dTheta( K, _e, _a2, _a1, alpha );
   numIter;

    pcl::PointXYZ p;
    p.x = _a1*pow( fabs(sin(alpha)), _e );
    p.y = _a2*pow( fabs(cos(alpha)), _e );
    p.z = 0;
    cloud_base.push_back(p);
 } while( alpha > 0 && numIter < maxIter );

 std::cout << "After final big stint: "<< cloud_base.points.size() << std::endl;

 // Put in final version
 double x_signs[4] = {-1,1,1,-1};
 double y_signs[4] = {1,1,-1,-1};

 pcl::PointCloud<pcl::PointXYZ>::iterator it;
 for( int i = 0; i < 4; ++i ) {
   for( it = cloud_base.begin(); it != cloud_base.end(); ++it ) {
     pcl::PointXYZ p;
     p.x = x_signs[i]*(*it).x;
     p.y = y_signs[i]*(*it).y;
     p.z = (*it).z;
     cloud->points.push_back(p);
   }
 }
  cloud->width = 1;
  cloud->height = cloud->points.size();

  return cloud;
}


