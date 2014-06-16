/**
 * @file sampleSQ.cpp
 * @brief Generate pointcloud with Super Quadric sampled
 * @author A. Huaman Quispe
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <unistd.h>
#include <stdlib.h> 

#include "SQ_utils.h"

void printHelp();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    // Parse the input arguments
    int v;
    double a,b,c;
    double e1,e2;
    double x,y,z, ra,pa,ya;
    std::string filename;

    // Initialize them
    a = 0.5; b = 0.5; c = 0.5;
    e1 = 0.5; e2 = 0.5;
    x = 0; y = 0; z = 0;
    ra = 0.0; pa = 0; ya = 0; 
    filename = std::string("sampleSQ_output.pcd");

    opterr = 0;
    while( (v = getopt(argc, argv, "N:a:b:c:x:y:z:R:P:Y:e:f:h")) != -1 ) {
	
	switch(v) {
	case 'N': {
	    filename = std::string( optarg );
	} break;
	case 'a': {
	    a = atof( optarg );
	} break;
	case 'b': {
	    b = atof( optarg );
	} break;
	case 'c': {
	    c = atof( optarg );
	} break;
	case 'x': {
	    x = atof( optarg );
	} break;
	case 'y': {
	    y = atof( optarg );
	} break;
	case 'z': {
	    z = atof( optarg );
	} break;
	case 'R': {
	    ra = atof( optarg );
	} break;
	case 'P': {
	    pa = atof( optarg );
	} break;
	case 'Y': {
	    ya = atof( optarg );
	} break;
	case 'e': {
	    e1 = atof( optarg );
	} break;
	case 'f': {
	    e2 = atof( optarg );
	} break;
	case 'h': {
	  printHelp();
	  return 1;
	} break;
	} // end of switch

    } // end of while


    // Sample SQ naive
    SQ_parameters par; 
    par.dim[0] = a; par.dim[1] = b; par.dim[2] = c;
    par.e[0] = e1; par.e[1] = e2;
    par.trans[0] = x; par.trans[1] = y; par.trans[2] = z;
    par.rot[0] = ra; par.rot[1] = pa; par.rot[2] = ya;
    
    // See values
    std::cout <<"\t ** Sampled SQ Information: **"<< std::endl;
    printParamsInfo( par );

    // Generate samples
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
    cloud = sampleSQ_uniform( par );
    
    // Save
    pcl::io::savePCDFileASCII( filename, *cloud );        
    std::cout <<"\t [*] Saved "<< filename  << std::endl;

    return 0;
}

/**
 * @function printHelp
 */
void printHelp() {
  std::cout <<" Syntax: ./sampleSQ  -a A -b B -c C -e E1 -f E2 -x X -y Y -z Z -R roll -P pitch -Y yaw -N filename.pcd "<< std::endl;
  return;
}
