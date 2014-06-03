/**
 * @file SQ_fitter_test.cpp
 */
#include <iostream>
#include <SQ_fitter.h>
#include <pcl/io/pcd_io.h>


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  if( argc != 2 ) {
    std::cout << "Syntax: "<<argv[0]<<" file.pcd"<< std::endl;
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  if( pcl::io::loadPCDFile<pcl::PointXYZ>( std::string(argv[1]),
					   * cloud ) == -1 ) {
    std::cout << "\t [ERROR] Could not read pointcloud"<< std::endl;
    return 1;
  }
  
  SQ_fitter<pcl::PointXYZ> sqf;
  double smax, smin, thresh; int N;

  smax = 0.05; smin = 0.01;
  thresh = 0.1;
  N = 5;
  
  if( sqf.SQFitting( cloud,
		     smax, smin,
		     N, thresh ) ) {
    std::cout << "\t [*] Fitting was successful" << std::endl;
  } else {
    std::cout << "\t [*] Fitting was unsuccessful" << std::endl;
  }
  
  
  return 0;
}
