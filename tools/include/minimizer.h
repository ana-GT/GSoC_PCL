/**
 * @file minimizer.h
 * @brief Draft version of minimizer with Maxima symbolic equations in .f file
 */
#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Helpers
double getRand( double _min, double _max );

extern "C" {
    void jac_( double *a, double *b, double *c, 
	       double *e1, double *e2,
	      double *x, double *y, double *z, 
	      double *J );

    void hessian_( double *a, double *b, double *c, 
		   double *e1, double *e2,
		   double *x, double *y, double *z, 
		   double *H );
}

/**
 * @class minimizer
 */
class minimizer {

 public:
    minimizer();
    ~minimizer();

    void generatePoints( int _num,
			 double _a1, double _a2, double _a3,
			 double _e1, double _e2 );
    bool loadPoints( std::string _pcdFilename );
    void visualizePoints();
    bool minimize();
    Eigen::VectorXd df( Eigen::VectorXd _params );
    Eigen::MatrixXd ddf( Eigen::VectorXd _params ); 
    int getNumSamples() { return mSamples->points.size(); }
 private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr mSamples;

    Eigen::VectorXd mParams; // Coefficients to be found
    double mLambda; // Damped coefficient

    double mMinThresh;
    int mMaxIter;
};
