/**
 * @file fitSegmentedPointcloud
 * @brief Test fitter of Duncan's 2013 ICRA Paper
 */
#include <SQFitting.h>
#include <SQTypes.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <iostream>


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    ope::SQFitting sqf;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    ope::SQParameters initParams;
    ope::SQParameters bestParams;


	if( argc != 2 ) {
		std::cout << "Syntax: "<< argv[0] <<" segmentedPointcloud.pcd"<< std::endl;	
		return -1;
	}


  if( pcl::io::loadPCDFile<pcl::PointXYZ>( argv[1], cloud ) == -1 ) {
    std::cout <<"Could not read file "<< argv[1] << std::endl;   
    return -1;
  }

   sqf.estimateInitialParameters( cloud, initParams );

    // Print init params
    std::cout << "Initial SQ parameters:"<< std::endl;
    std::cout << initParams;
    std::cout << std::endl;

    sqf.performShapeFitting( cloud, 
			     initParams,
			     bestParams );


    // Print bestParams
    std::cout << "Best SQ parameters:"<< std::endl;
    std::cout << bestParams;
    std::cout << std::endl;

    return 0;
}
