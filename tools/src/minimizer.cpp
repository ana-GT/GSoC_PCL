/**
 * @file minimizer.cpp
 * @brief Mock minimizer for fi =a0 +a1*xi^2 + a2*yi + a3*zi^3 
 */
#include "minimizer.h"
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <pcl/visualization/pcl_visualizer.h>

/**
 * @function minimizer
 * @brief Constructor
 */
minimizer::minimizer() :
    mSamples( new pcl::PointCloud<pcl::PointXYZ> ) {
    srand( time(NULL) );

    mParams.resize(5);
    mParams << 1.0, 4.5, 3.6, 1.5, 0.5;
    mLambda = 0.1;
    mMaxIter = 10000;
    mMinThresh = 0.001;
}

/**
 * @function minimizer
 * @brief Destructor
 */
minimizer::~minimizer() {

}



/**
 * @function generatePoints
 */
void minimizer::generatePoints( int _num,
				double _a1, double _a2, double _a3,
				double _e1, double _e2 ) {

    double x, y, z, f;

    mSamples->points.resize(0);
    mF.resize(0);
    for( int i = 0; i < _num; ++i ) {
	x = getRand( -1, 1 );
	y = getRand( -1, 1 );
	z = getRand( -1, 1 );

	f = mParams(0) + mParams(1)*x*x + mParams(2)*y + mParams(3)*z*z*z + getRand( -0.2, 0.2 );

	mSamples->points.push_back( pcl::PointXYZ(x,y,z) );
	mF.push_back(f);
    }
   
}

/**
 * @function visualizePoints
 */
void minimizer::visualizePoints() {

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0, "main", 0);
    viewer->initCameraParameters ();
    
    // 2. Visualize the clouds
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color( mSamples, 255, 0, 0 );
    viewer->addPointCloud<pcl::PointXYZ> ( mSamples, color, "cloud" );	
       
    // 3. Run loop
    while (!viewer->wasStopped ()) {
	viewer->spinOnce (100);
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }    

}

/**
 * @function minimize
 */
bool minimizer::minimize() {
    
    Eigen::MatrixXd H; 
    Eigen::MatrixXd dH = Eigen::MatrixXd::Identity(5,5);

    // Initialize mA with some value
    int iter = 0;
    Eigen::VectorXd oldParams;

    mParams << 1.0, 1.0, 1.0, 1.0, 1.0;
    std::cout << "Initial guess for coefficients: "<< mParams.transpose() << std::endl;

    do {
	oldParams = mParams;
	H = ddf( mParams );
	for( int i = 0; i < 5; ++i ) { dH(i,i) = H(i,i); }
	mParams = mParams - (H + mLambda*dH).inverse()*df( mParams );
	
	iter++;
    } while( iter < mMaxIter && (mParams - oldParams).norm() > mMinThresh );
    
    if( iter >= mMaxIter ) {
	std::cout << "[BAD] Crab, we did not converge after "<<iter<<" iterations"<< std::endl;
	std::cout << "Final coefficients: "<< mParams.transpose() << std::endl;
	return false;
    } else {
	std::cout << "[GOOD] Yes, we did converge in "<< iter << std::endl;
	std::cout << "Final coefficients: "<< mParams.transpose() << std::endl;
	return true;
    }
    

    return true;
}

/**
 * @function df
 * @brief Returns the derivative of f w.r.t. params
 */
Eigen::VectorXd minimizer::df( Eigen::VectorXd _params ) {
    
    Eigen::VectorXd df(5); df = Eigen::VectorXd::Zero(5);
    double x, y, z;
    
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    int i;
    double* jac;

    for( int n = 0; n < 5; ++n ) {
	df(n) = 0;	
    }

    for( it = mSamples->begin(), i = 0; 
	 it != mSamples->end(); 
	 ++it, ++i ) {
	Jac_( _params(0), _params(1), _params(2), _params(3), _params(4),
	      it->x, it->y, it->z, jac );

	for( int n = 0; n < 5; ++n ) {
	    df(n) += jac[n];	
	}

    }
    
    return df;
    
}

/**
 * @function ddf
 * @brief Calculates the Hessian
 */
Eigen::MatrixXd minimizer::ddf( Eigen::VectorXd _params ) {

    Eigen::MatrixXd ddf(5,5); ddf = Eigen::MatrixXd::Zero(5,5);
    double x, y, z;

    
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    int i;
    double **hes;

    for( int m = 0; m < 5; ++m ) {
	for( int n = 0; n < 5; ++n ) {
	    ddf(m,n) = 0;
	}
    }


    for( it = mSamples->begin(), i = 0; 
	 it != mSamples->end(); 
	 ++it, ++i ) {

	Hessian_( _params(0), _params(1), _params(2),
		  _params(3), _params(4),
		  it->x, it->y, it->z, hes );

	for( int m = 0; m < 5; ++m ) {
	    for( int n = 0; n < 5; ++n ) {
		ddf(m,n) += hes[m][n];
	    }
	}

    }


    return ddf;
}

/**
 * @function getRand
 */
double getRand( double _min,
		double _max ) {
    return _min + (_max - _min)*( (double) rand() / (double) RAND_MAX );   
}
