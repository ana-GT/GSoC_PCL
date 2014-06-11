/*********************************************/
/* -*- mode:C++; c-basic-offset: 4 -*-       */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*********************************************/

#include "SQ_sampler.h"
#include "SQ_utils.h"
#include <pcl/common/transforms.h>

/**
 * @function SQ_sampler
 * @brief Constructor
 */
SQ_sampler::SQ_sampler() {

}

/**
 * @function ~SQ_sampler
 * @brief Destructor
 */
SQ_sampler::~SQ_sampler() {

}

/**
 * @function getSuperEllipse
 * @brief Outputs a superellipse (2D shape) in the XY plane
 */
pcl::PointCloud<pcl::PointXYZ> SQ_sampler::getSuperEllipse( const double &_a1,
							    const double &_a2,
							    const double &_e,
							    const int &_numSamples ) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    double x, y;
    double dang = 2*M_PI / _numSamples;
    double c, s;

    double ang = -1*M_PI;
    for( int i = 0; i < _numSamples; ++i ) {

	c = cos(ang); s = sin(ang);
	x = _a1*pow( fabs(c), _e );
	y = _a2*pow( fabs(s), _e );

	if( c < 0 ) { x = x*(-1); }
	if( s < 0 ) { y = y*(-1); }

	pcl::PointXYZ p;
	p.x = x; p.y = y; p.z = 0;
	cloud.points.push_back(p);

	ang += dang;
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;

    return cloud;
}

/**
 * @function getSE_Pilu_Fisher 
 * @brief Get Super Ellipse using the method of Pilu and Fisher for uniform sampling
 * @brief https://homepages.inf.ed.ac.uk/rbf/MY_DAI_OLD_FTP/rp764.pdf
 */
pcl::PointCloud<pcl::PointXYZ> SQ_sampler::getSuperEllipse_Pilu_Fisher( const double &_a1,
									const double &_a2,
									const double &_e,
									const int &_numSamples ) {
    
    pcl::PointCloud<pcl::PointXYZ> cloud;
    double x, y;
    double c; double s;
    double D = 0.005;

    double theta; double dtheta;

    bool ascending = true;
    theta = 0.0;
    for( int i = 0; i < _numSamples; ++i ) {

	c = cos(theta); s = sin(theta);
	if( !ascending ) { c = sin(theta); s = cos(theta); }

	x = _a1*pow( c, _e );
	y = _a2*pow( s, _e );

	pcl::PointXYZ p;
	p.x = x; p.y = y; p.z = 0;
	cloud.points.push_back(p);

	dtheta = diff_theta( D, _a1, _a2, _e, theta, ascending );
	if( ascending ) { theta += fabs(dtheta); }
	else { theta -= fabs(dtheta); }

	if( theta >= M_PI/2.0) { std::cout << "Exceeded 90"<< std::endl; break; }
	if( theta <= 0 && !ascending ) { std::cout << "Descending and got zero" << std::endl; break; }

	std::cout << " Point: "<< p.x << " "<< p.y << " "<< p.z << 
	    " theta: "<< theta << " dtheta: "<< dtheta << 
	    " ascending: "<< ascending <<std::endl;

    }

    // Now flip to the second quadrant
    int n = cloud.points.size();
    for( int i = 0; i < n; ++i ) {
	pcl::PointXYZ p;
	p = cloud.points[i];
	p.x = -1*p.x;
	cloud.points.push_back( p );
    }

    // Now mirror to III and IV quadrants
    n = cloud.points.size();
    for( int i = 0; i < n; ++i ) {
	pcl::PointXYZ p;
	p = cloud.points[i];
	p.y = -1*p.y;
	cloud.points.push_back( p );
    }
    
    
    cloud.width = cloud.points.size();
    cloud.height = 1;

    return cloud;
}

/**
 * @function diff_theta
 */
double SQ_sampler::diff_theta( const double &_D, 
			       const double &_a1, 
			       const double &_a2,
			       const double &_e,
			       const double &_theta,
			       bool &_ascending ) {
    double num;
    double den; double den1; double den2;
    double c; double s;

    double thresh_0 = 0.01;
    double thresh_pi_2 = M_PI/2.0 - 0.1;

    c = cos(_theta); s = sin(_theta);
    
    if( _theta < thresh_0 ) {	
	return pow( (_D/_a2) - pow(_theta,_e), (1.0/_e) ) - _theta;
	 } else if( _theta > thresh_pi_2 ) {	
	double theta = M_PI / 2.0 - _theta;
	_ascending = false;
	return pow( (_D/_a1) - pow( theta,_e), (1.0/_e) ) - theta;  
    } else {

	num = c*c*s*s;
	den1 = _a1*_a1*pow(s,4)*pow(pow(c,_e),2);
	den2 = _a2*_a2*pow(c,4)*pow(pow(s,_e),2);
	den = den1 + den2;
	return (_D/_e)*sqrt( num / den );
    }
}


/**
 * @function sampleSQ_naive
 * @brief Sample n \in [-PI/2, PI/2] and w \in [-PI,PI>
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr SQ_sampler::sampleSQ_naive( SQ_params _par ) {

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
    a = _par.a; b = _par.b; c = _par.c; e1 = _par.e1; e2 = _par.e2;

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
    pcl::transformPointCloud<pcl::PointXYZ, double>( cloud_raw,
						     *cloud,
						     param2Transf(_par) );
    
    cloud->height = 1;
    cloud->width = cloud->points.size();

    return cloud;
}

