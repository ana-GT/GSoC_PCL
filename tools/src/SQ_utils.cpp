/**
 * @file SQ_utils.cpp
 */
#include "SQ_utils.h"
#include "SQ_sampler.h"
#include <iostream>


/**
 * @function param2Transf 
 * @brief Read a SQ_params instance and get the Tf 
 */
Eigen::Isometry3d param2Transf( const SQ_params &_par ) {

    Eigen::Isometry3d Tf = Eigen::Isometry3d::Identity();
    Tf.translation() << _par.px, _par.py, _par.pz;
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(_par.ya, Eigen::Vector3d::UnitZ() )*
	Eigen::AngleAxisd( _par.pa, Eigen::Vector3d::UnitY() )*
	Eigen::AngleAxisd( _par.ra, Eigen::Vector3d::UnitX() );
    Tf.linear() = rot;
    return Tf;
}

/**
 * @function transf2Params
 * @brief Fill par arguments for rot and trans from a _Tf
 */
void transf2Params( const Eigen::Isometry3d &_Tf,
		    SQ_params &_par ) {

    _par.px = _Tf.translation().x();
    _par.py = _Tf.translation().y();
    _par.pz = _Tf.translation().z();

    double r31, r11, r21, r32, r33;
    double r, p, y;

    r31 = _Tf.linear()(2,0);
    r11 = _Tf.linear()(0,0);
    r21 = _Tf.linear()(1,0);
    r32 = _Tf.linear()(2,1);
    r33 = _Tf.linear()(2,2);

    p = atan2( -r31, sqrt(r11*r11 + r21*r21) );
    y = atan2( r21 / cos(p), r11 / cos(p) );
    r = atan2( r32 / cos(p), r33 / cos(p) );

    _par.ra = r;
    _par.pa = p;
    _par.ya = y;
}

/**
 * @function params2Vec
 * @brief Creates a Eigen::VectorXd of size 11 to enter in minimizer
 */
Eigen::VectorXd params2Vec( const SQ_params &_par ) {

    Eigen::VectorXd vec(11);
    vec << _par.a, _par.b, _par.c,
	_par.e1, _par.e2,
	_par.px, _par.py, _par.pz,
	_par.ra, _par.pa, _par.ya;

    return vec;
}

/**
 * @function vec2Param
 * @brief Fill a param from a Eigen::VectorXd of size 11
 */
bool vec2Param( const Eigen::VectorXd &_vec,
		SQ_params &_par ) {

    if( _vec.size() != 11 ) { return false; }
    _par.a = _vec(0);
    _par.b = _vec(1);
    _par.c = _vec(2);

    _par.e1 = _vec(3);
    _par.e2 = _vec(4);

    _par.px = _vec(5);
    _par.py = _vec(6);
    _par.pz = _vec(7);

    _par.ra = _vec(8);
    _par.pa = _vec(9);
    _par.ya = _vec(10);

    return true;
}

/**
 * @function printParamsInfo
 * @brief Print in a human-friendly form the parameters info
 */
void printParamsInfo( const SQ_params &_par ) {

    std::cout << "\t ** Params: **"<< std::endl;
    std::cout << "\t Axis dimensions: "<< _par.a << ", "<< _par.b<<", "<< _par.c << std::endl;
    std::cout << "\t Epsilons: "<< _par.e1 << ", "<< _par.e2<< std::endl;
    std::cout << "\t Trans: "<< _par.px << ", "<< _par.py<<", "<< _par.pz << std::endl;
   std::cout << "\t Rot: "<< _par.ra << ", "<< _par.pa<<", "<< _par.ya << std::endl;
}


/***
 * @function visualizeSQ
 * @brief Add sampled Super Quadric to viewer (red color by default)
 */
void visualizeSQ(  boost::shared_ptr<pcl::visualization::PCLVisualizer> &_viewer,
		   const SQ_params &_par,
		   std::string _name,
		   int _r, int _g, int _b ) {

    // Create sample pointcloud of SQ expressed by _par
    SQ_sampler sqs;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sq_pcd( new pcl::PointCloud<pcl::PointXYZ>() );
    sq_pcd = sqs.sampleSQ_naive( _par );

    // Add it 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color( sq_pcd, _r, _g, _b );
    _viewer->addPointCloud<pcl::PointXYZ> ( sq_pcd, color, _name );	

}

/**
 * @function clamp
 * @brief Keep SQ parameters within reasonable values (positive dimensions
 * and limited e1 and e2) 
 */
bool clamp( SQ_params &_par ) {

    double min_d = 0.01;
    double max_d = 1.5;
    bool clamped = false;

    if( _par.e1 < 0.1 ) { _par.e1 = 0.1; clamped = true; }
    if( _par.e1 > 1.9 ) { _par.e1 = 1.9; clamped = true; }

    if( _par.e2 < 0.1 ) { _par.e2 = 0.1; clamped = true; }
    if( _par.e2 > 1.9 ) { _par.e2 = 1.9; clamped = true; }

    if( _par.a < min_d ) { _par.a = min_d; clamped = true; }
    if( _par.a > max_d ) { _par.a = max_d; clamped = true; }

    if( _par.b < min_d ) { _par.b = min_d; clamped = true; }
    if( _par.b > max_d ) { _par.b = max_d; clamped = true; }

    if( _par.c < min_d ) { _par.c = min_d; clamped = true; }
    if( _par.c > max_d ) { _par.c = max_d; clamped = true; }

    return clamped;
}

bool clamp( Eigen::VectorXd &_par ) {

    double min_d = 0.01;
    double max_d = 1.5;
    double min_e = 0.1;
    double max_e = 1.9;
    bool clamped = false;

    // a 0 b 1 c 2 e1 3 e2 4 

    if( _par(3) < min_e ) { _par(3) = min_e; clamped = true; }
    if( _par(3) > max_e ) { _par(3) = max_e; clamped = true; }

    if( _par(4) < min_e ) { _par(4) = min_e; clamped = true; }
    if( _par(4) > max_e ) { _par(4) = max_e; clamped = true; }

    if( _par(0) < min_d ) { _par(0) = min_d; clamped = true; }
    if( _par(0) > max_d ) { _par(0) = max_d; clamped = true; }

    if( _par(1) < min_d ) { _par(1) = min_d; clamped = true; }
    if( _par(1) > max_d ) { _par(1) = max_d; clamped = true; }

    if( _par(2) < min_d ) { _par(2) = min_d; clamped = true; }
    if( _par(2) > max_d ) { _par(2) = max_d; clamped = true; }

    return clamped;

}
