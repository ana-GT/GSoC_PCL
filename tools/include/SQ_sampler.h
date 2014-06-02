/**
 * @file SQ_sampler.h
 */
#pragma once 

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "SQ_params.h"

/**
 * @class SQ_sampler
 */
class SQ_sampler {

 public:
    SQ_sampler();
    ~SQ_sampler();

    pcl::PointCloud<pcl::PointXYZ> getSuperEllipse( const double &_a1,
						    const double &_a2,
						    const double &_e,
						    const int &_numSamples = 10000 );


    pcl::PointCloud<pcl::PointXYZ> getSuperEllipse_Pilu_Fisher( const double &_a1,
								const double &_a2,
								const double &_e,
								const int &_numSamples = 10000 );


    double diff_theta( const double &_D, 
		       const double &_a1, 
		       const double &_a2,
		       const double &_e,
		       const double &_theta,
		       bool &_ascending );    

    pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSQ_naive( SQ_params _par );
    
    
 private:

};
