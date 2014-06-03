/**
 * @file SQ_utils.cpp
 */
#include "SQ_utils.h"
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
