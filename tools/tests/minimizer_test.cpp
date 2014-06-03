/**
 * @file minimizeDraftTest.cpp
 * @brief Minimize Draft code test version
 */

#include "minimizer.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    if( argc != 2 ) {
	std::cout << "Usage: "<< argv[0] << " file.pcd" << std::endl;
	return 1;
    }

    minimizer mM;
    if( !mM.loadPoints( std::string(argv[1]) ) ) {
	std::cout << "[ERROR] Could not load the points correctly"<< std::endl;
	return 1;
    }

    //mM.visualizePoints();

    std::cout << "\t [INFO] Calling minimization!"<< std::endl;
    SQ_params par_in, par_out;

    if( mM.minimize(par_in, par_out) ) {
	std::cout << "\t SUCCESS!"<< std::endl; 
    } else {
	std::cout << "\t FAILURE!"<< std::endl;
    }
    
    return 0;
}
