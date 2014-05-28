/**
 * @file minimizeTest.cpp
 */

#include "minimizer.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    minimizer mM;
    mM.generatePoints(100);

    //mM.visualizePoints();

    std::cout << "\t [INFO] Calling minimization!"<< std::endl;
    if( mM.minimize() ) {
	std::cout << "\t SUCCESS!"<< std::endl; 
    } else {
	std::cout << "\t FAILURE!"<< std::endl;
    }
    
    return 0;
}
