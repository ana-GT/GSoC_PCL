/**
 * @file minimizer.h
 * @brief Draft version of minimizer with Maxima symbolic equations in .f file
 */
#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "SQ_utils.h"

#include "matlab_equations.h"

// Helpers
double getRand( double _min, double _max );

extern "C" {
    void jac_( double *a, double *b, double *c, 
	       double *e1, double *e2,
	       double *px, double *py, double *pz,
	       double *ra, double *pa, double *ya,
	       double *x, double *y, double *z, 
	       double *J );

    void hessian_( double *a, double *b, double *c, 
		   double *e1, double *e2,
		   double *px, double *py, double *pz,
		   double *ra, double *pa, double *ya,
		   double *x, double *y, double *z, 
		   double *H );

    void error_( double *a, double *b, double *c, 
		 double *e1, double *e2,
		 double *px, double *py, double *pz,
		 double *ra, double *pa, double *ya,
		 double *x, double *y, double *z, 
		 double *er );

}

/**
 * @class minimizer
 */
class minimizer {

 public:
    minimizer();
    ~minimizer();

    bool loadPoints( std::string _pcdFilename );
    bool loadPoints( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud );
    void visualizePoints();
    bool minimize( const SQ_params &_par_in,
		   SQ_params &_par_out );
    Eigen::VectorXd df( Eigen::VectorXd _params );
    Eigen::MatrixXd ddf( Eigen::VectorXd _params ); 
    int getNumSamples() { return mSamples->points.size(); }
 private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr mSamples;

    Eigen::VectorXd mParams; // Coefficients to be found
    double mLambda; // Damped coefficient

    double mMinThresh;
    int mMaxIter;
    int mNumParams;
};
