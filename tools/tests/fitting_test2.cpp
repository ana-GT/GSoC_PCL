/**
 * @file fitting_test2.cpp
 * @brief Test functions one by one
 */
#include <iostream>
#include <SQ_fitter.h>
#include <SQ_sampler.h>
#include <SQ_params.h>

#include <pcl/io/pcd_io.h>

#include <random>


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    // 0. Initialize random seed
    std::random_device rd;
    std::mt19937 gen( rd() );
    std::uniform_real_distribution<> dis(-0.1, 0.1 );


    // 1. Sample a SQ (pick any parameters)
    SQ_params par;
    par.a = 1.2; par.b = 2.4; par.c = 1.9;
    par.e1 = 0.8; par.e2 = 0.8;
    par.px = 0.1; par.py = 0.9; par.pz = 0.4;
    par.ra = 0.5; par.pa = 0.7; par.ya = 0.7;


    SQ_sampler sqs;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>());
    cloud = sqs.sampleSQ_naive( par );

    // 1.a. Let's add some error and see how bad it goes (error-related)
    for( int i = 0; i < cloud->points.size(); ++i ) {
	cloud->points[i].x += dis(gen);
  	cloud->points[i].y += dis(gen);
	cloud->points[i].z += dis(gen);
    }


    
    SQ_fitter<pcl::PointXYZ>sqf;    
    
    if( sqf.SQFitting( cloud,
		       0.05, 0.01,
		       4, 0.01 ) ) {
	std::cout << "FOUND A SOLUTION"<< std::endl;
    } else {
	std::cout << "BANG, DID NOT FIND A SOLUTION" << std::endl;
	}
    
    

  
  return 0;
}
