/*********************************************/
/* -*- mode:C++; c-basic-offset: 4 -*-       */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*********************************************/

#include "SQ_sampler.h"

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
 * @function getSuperQuadric
 */
pcl::PointCloud<pcl::PointXYZ> SQ_sampler::getSuperQuadric( const double &_a1,
							    const double &_a2,
							    const double &_a3,
							    const double &_e1,
							    const double &_e2,
							    const int &_numSamples ) {

    pcl::PointCloud<pcl::PointXYZ> cloud;

    double n; double w;    
    double dn; double dw;
    int num_ni = 100;
    int num_wi = 100;
    double x, y, z;
    double cn, sn, cw, sw;

    // For
    n = -0.5*M_PI;
    w = -1*M_PI;

    dn = M_PI / num_ni;
    dw = 2.0*M_PI / num_wi;
    
    for( int ni = 0; ni < num_ni; ++ni ) {	
	n += dn;
	cn = cos(n); sn = sin(n);

	w = -1*M_PI;
	for( int wi = 0; wi < num_wi; ++wi ) {
	    w += dw;
	    
	    cw = cos(w); sw = sin(w);
	    x = _a1*pow( fabs(cn), _e1 )*pow( fabs(cw), _e2 );
	    y = _a2*pow( fabs(cn), _e1 )*pow( fabs(sw), _e2 );
	    z = _a3*pow( fabs(sn), _e1 );

	    if( cn*cw < 0 ) { x = -1*x; }
	    if( cn*sw < 0 ) { y = -1*y; }
	    if( sn < 0 ) { z = -1*z; }

	    pcl::PointXYZ p;	    
	    p.x = x; p.y = y; p.z = z;
	    cloud.points.push_back( p );
	}
	
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;


    return cloud;
}

