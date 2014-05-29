/**
 * @file minimizer.h
 * @brief Mock minimizer for fi =a0 +a1*xi^2 + a2*yi + a3*zi^3 
 */
#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Helpers
double getRand( double _min, double _max );

/**
 * @class minimizer
 */
class minimizer {

 public:
    minimizer();
    ~minimizer();

    void generatePoints( int _num );
    void visualizePoints();
    bool minimize();
    Eigen::VectorXd df( Eigen::VectorXd _a );
    Eigen::MatrixXd ddf( Eigen::VectorXd _a ); 
    int getNumSamples() { return mSamples->points.size(); }
 private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr mSamples;
    std::vector<double> mF;

    Eigen::VectorXd mA; // Coefficients to be found
    double mLambda; // Damped coefficient

    double mMinThresh;
    int mMaxIter;
};
