/**
 * @file minimizer.cpp
 * @brief Mock minimizer for fi =a0 +a1*xi^2 + a2*yi + a3*zi^3 
 */
#include "minimizer.h"
#include "SQ_sampler.h"
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "matlab_equations.h"

/**
 * @function minimizer
 * @brief Constructor
 */
minimizer::minimizer() :
    mSamples( new pcl::PointCloud<pcl::PointXYZ> ) {
    srand( time(NULL) );

    mLambda = 0.01;
    mMaxIter = 1000;
    mMinThresh = 0.005;

    // 11 Parameters: a,b,c, e1,e2, px,py,pz, ra,pa,ya
    mNumParams = 11;
    mParams.resize(mNumParams);
}

/**
 * @function minimizer
 * @brief Destructor
 */
minimizer::~minimizer() {

}

/**
 * @function loadPoints
 */
bool minimizer::loadPoints( std::string _pcdFilename ) {
    
    if( pcl::io::loadPCDFile<pcl::PointXYZ>( _pcdFilename.c_str(), *mSamples ) == -1 ) {
	std::cout <<"\t [ERROR] Could not read file " << std::endl;
	return false;
    }

    std::cout << "\t [GOOD] Loaded "<< mSamples->points.size() << " points" << std::endl;
    return true;
    
}

/**
 * @function loadPoints
 */
bool minimizer::loadPoints( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud ) {
    
    mSamples = _cloud;
    std::cout << "\t [GOOD] Loaded "<< mSamples->points.size() << " points" << std::endl;
    return true;    
}


/**
 * @function visualizePoints
 */
void minimizer::visualizePoints() {

    std::cout << "\t [INFO] Visualizing input cloud to minimizer"<< std::endl;
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
bool minimizer::minimize( const SQ_params &_par_in,
			  SQ_params &_par_out ) {
    
    Eigen::MatrixXd H;  Eigen::MatrixXd J;
    Eigen::MatrixXd dH = Eigen::MatrixXd::Identity( mNumParams, mNumParams );
    double error;

    // Initialize mA with some value
    int iter = 0;
    Eigen::VectorXd oldParams;
    
    mParams << params2Vec( _par_in );

    std::cout << "Initial guess for coefficients: "<< mParams.transpose() << std::endl;

    do {
	oldParams = mParams;
    std::cout << "Starting calculate H and F"<< std::endl;
	double ts, tf, dt;
    ts = clock();
	H = ddf( mParams );
    tf = clock();
    dt = (tf - ts) / CLOCKS_PER_SEC;
    std::cout << "Hessian time: "<< dt << std::endl;
    ts = clock();
	J = df( mParams );
    tf = clock();
    dt = (tf - ts) / CLOCKS_PER_SEC;
    std::cout << "Jac time: "<< dt << std::endl;

    std::cout << "Ending calculate H and F"<< std::endl;

	for( int i = 0; i < mNumParams; ++i ) { dH(i,i) = H(i,i); }
	mParams = mParams - (H + mLambda*dH).inverse()*J;
	//clamp(mParams);
	iter++;
	error = (mParams - oldParams).norm();
	for( int i = 0; i < mNumParams; ++i ) {
	    if( mParams(i) != mParams(i) ) {
		std::cout << "Param "<<i<<" is NAN. Return false"<< std::endl;
		return false;
	    }
	}

	std::cout << "Iter: "<< iter << " with error: "<< error << std::endl;

    } while( iter < mMaxIter && error > mMinThresh );
    
    vec2Param( mParams, _par_out );
    
    
    if( iter >= mMaxIter ) {
	std::cout << "[BAD] Crab, we did not converge after "<<iter<<" iterations"<< std::endl;
	std::cout << "Final coefficients: "<< mParams.transpose() << std::endl;
	return false;
    } else {
	std::cout << "[GOOD] Yes, we did converge in "<< iter << std::endl;
	std::cout << "Final coefficients: "<< mParams.transpose() << std::endl;
	std::cout << "J: \n"<< J << std::endl; 
	std::cout << "H: \n" << H << std::endl;
	return true;
    }
    

    return true;
}

/**
 * @function df
 * @brief Returns the derivative of f w.r.t. params
 */
Eigen::VectorXd minimizer::df( Eigen::VectorXd _params ) {
    
    Eigen::VectorXd df = Eigen::VectorXd::Zero(mNumParams);
    double x, y, z;
    
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    int i;
    Eigen::VectorXd jac;


    for( it = mSamples->begin(); it != mSamples->end(); ++it ) {
	
	x = it->x; y = it->y; z = it->z;
	
	jac = jac_MATLAB( _params, x, y, z ); 

	for( int n = 0; n < mNumParams; ++n ) {
	    if( jac[n] != jac[n] ) {
		std::cout << "[Jacobian] NAN value in ("<<n<<")"<< std::endl;
	    } 
	}
	df += jac;
    }
    
    return df;    
}

/**
 * @function ddf
 * @brief Calculates the Hessian
 */
Eigen::MatrixXd minimizer::ddf( Eigen::VectorXd _params ) {

    Eigen::MatrixXd ddf = Eigen::MatrixXd::Zero(mNumParams,mNumParams);
    
    pcl::PointCloud<pcl::PointXYZ>::iterator it;    
    double hest[11][11];

    double x, y, z;
        
    for( it = mSamples->begin(); it != mSamples->end(); ++it ) {

	x = it->x; y = it->y; z = it->z;
	hess_MATLAB( _params, x, y, z, 
		     hest );

	int ind = 0;
	for( int m = 0; m < mNumParams; ++m ) {
	    for( int n = 0; n < mNumParams; ++n ) {
		
		if( hest[m][n] != hest[m][n] ) {
		    std::cout << "[Hessian] NAN value in ("<< m<<", "<<n <<")"<< " with params:"<< _params.transpose() << " and xyz:"<<x<<", "<<y<<", "<<z<< std::endl;
		} else {
		    ddf(m,n) += hest[m][n];
		}
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
