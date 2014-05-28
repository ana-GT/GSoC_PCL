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

    mA.resize(4);
    mA << 1.0, 4.5, 3.6, 2.7;
    mLambda = 0.1;
    mMaxIter = 10000;
    mMinThresh = 0.001;
}

minimizer::~minimizer() {

}

/**
 * @function generatePoints
 */
void minimizer::generatePoints( int _num ) {

    double x, y, z, f;

    mSamples->points.resize(0);
    mF.resize(0);
    for( int i = 0; i < _num; ++i ) {
	x = getRand( -1, 1 );
	y = getRand( -1, 1 );
	z = getRand( -1, 1 );

	f = mA(0) + mA(1)*x*x + mA(2)*y + mA(3)*z*z*z + getRand( -0.2, 0.2 );

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
    viewer->addCoordinateSystem (1.0, 0);
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

    Eigen::MatrixXd H; Eigen::MatrixXd dH = Eigen::MatrixXd::Identity(4,4);

    // Initialize mA with some value
    int iter = 0;
    Eigen::VectorXd oldA;

    mA << 1.0, 1.0, 1.0, 1.0;
    std::cout << "Initial guess for coefficients: "<< mA.transpose() << std::endl;

    do {
	oldA = mA;
	H = ddf( mA );
	for( int i = 0; i < 4; ++i ) { dH(i,i) = H(i,i); }
	mA = mA - (H + mLambda*dH).inverse()*df( mA );
	
	iter++;
    } while( iter < mMaxIter && (mA - oldA).norm() > mMinThresh );
    
    if( iter >= mMaxIter ) {
	std::cout << "[BAD] Crab, we did not converge after "<<iter<<" iterations"<< std::endl;
	std::cout << "Final coefficients: "<< mA.transpose() << std::endl;
	return false;
    } else {
	std::cout << "[GOOD] Yes, we did converge in "<< iter << std::endl;
	std::cout << "Final coefficients: "<< mA.transpose() << std::endl;
	return true;
    }

}

/**
 * @function df
 * @brief Returns the derivative of f w.r.t. a
 */
Eigen::VectorXd minimizer::df( Eigen::VectorXd _a ) {

    Eigen::VectorXd dfda(4); dfda = Eigen::VectorXd::Zero(4);
    double x, y, z;
    double hr;

    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    int i;
    for( it = mSamples->begin(), i = 0; 
	 it != mSamples->end(); 
	 ++it, ++i ) {
	hr = _a(0) + _a(1)*pow( it->x, 2 ) + _a(2)*(it->y) + _a(3)*pow(it->z, 3 ) - mF[i];
	dfda(0) += hr;
	dfda(1) += hr*pow(it->x, 2);
	dfda(2) += hr*it->y;
	dfda(3) += hr*pow(it->z,3);	
    }

    return dfda;
}

/**
 * @function ddf
 * @brief Calculates the Hessian
 */
Eigen::MatrixXd minimizer::ddf( Eigen::VectorXd _a ) {

    Eigen::MatrixXd ddfda(4,4); ddfda = Eigen::MatrixXd::Zero(4,4);
    double x, y, z;
    double hr;

    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    int i;
    for( it = mSamples->begin(), i = 0; 
	 it != mSamples->end(); 
	 ++it, ++i ) {

	ddfda(0,0) += 1;
	ddfda(0,1) += pow( it->x, 2 );
	ddfda(0,2) += it->y;
	ddfda(0,3) += pow( it->z, 3 );

	//ddfda(1,0) += ;
	ddfda(1,1) += pow(it->x, 4);
	ddfda(1,2) += pow(it->x, 2)*(it->y);
	ddfda(1,3) += pow(it->x, 2)*pow(it->z, 3);

	//ddfda(2,0) += ;
	//ddfda(2,1) += ;
	ddfda(2,2) += pow(it->y, 2);
	ddfda(2,3) += pow(it->z,3)*(it->y);

	//ddfda(3,0) += ;
	//ddfda(3,1) += ;
	//ddfda(3,2) += ;
	ddfda(3,3) += pow(it->z,6);
    }

    ddfda(1,0) = ddfda(0,1);
    ddfda(2,0) = ddfda(0,2);
    ddfda(2,1) = ddfda(1,2);
    ddfda(3,0) = ddfda(0,3);
    ddfda(3,1) = ddfda(1,3);
    ddfda(3,2) = ddfda(2,3);

    return ddfda;
}

/**
 * @function getRand
 */
double getRand( double _min,
		double _max ) {
    return _min + (_max - _min)*( (double) rand() / (double) RAND_MAX );   
}
