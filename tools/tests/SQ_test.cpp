/**
 * @file SQ_test.cpp
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

#include "SQ_sampler.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    // Create the sampler class
    SQ_sampler sqs;

    // Store superellipsoid
    pcl::io::savePCDFileASCII( "se_cloud_01.pcd", 
			       sqs.getSuperEllipse( 1.5, 0.5, 0.1 ) );

    pcl::io::savePCDFileASCII( "se_pilu_cloud_01.pcd", 
			       sqs.getSuperEllipse_Pilu_Fisher( 1.5, 0.5, 0.1 ) );

    pcl::io::savePCDFileASCII( "se_pilu_cloud_02.pcd", 
			       sqs.getSuperEllipse_Pilu_Fisher( 1.5, 1.5, 0.2 ) );

    pcl::io::savePCDFileASCII( "se_cloud_05.pcd", 
			       sqs.getSuperEllipse( 1.5, 0.5, 0.5 ) );

    pcl::io::savePCDFileASCII( "se_cloud_10.pcd", 
			       sqs.getSuperEllipse( 1.5, 0.5, 1.0 ) );

    pcl::io::savePCDFileASCII( "se_cloud_20.pcd", 
			       sqs.getSuperEllipse( 1.5, 0.5, 2.0 ) );

    pcl::io::savePCDFileASCII( "sq_cloud_1_01.pcd",
			       sqs.getSuperQuadric( 1.0, 1.0, 1.0, 0.1, 1, 10000 ) );


    std::cout <<"Finished SQ_test.cpp "<< std::endl;

    return 0;
}
