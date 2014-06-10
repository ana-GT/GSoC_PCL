/**
 * @file minimizer_test.cpp
 * @brief Minimize Draft code test version
 */

#include "minimizer.h"
#include <SQ_sampler.h>

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    // 0. Initialize random seed
    std::random_device rd;
    std::mt19937 gen( rd() );
    std::uniform_real_distribution<> dis(-0.01, 0.015 );


    // 1. Sample a SQ (pick any parameters)
    SQ_params par;
    par.a = 0.5; par.b = 0.35; par.c = 0.2;
    par.e1 = 0.4; par.e2 = 0.2;
    par.px = 0.1; par.py = 0.9; par.pz = 0.4;
    par.ra = 0.5; par.pa = 0.7; par.ya = 0.5;

    SQ_sampler sqs;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>());
    cloud = sqs.sampleSQ_naive( par );

    // 1.a. Let's add some error and see how bad it goes (error-related)
    for( int i = 0; i < cloud->points.size(); ++i ) {
	cloud->points[i].x += dis(gen);
  	cloud->points[i].y += dis(gen);
	cloud->points[i].z += dis(gen);
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
   

    // 2. Apply LM in one level
    minimizer mM;
    mM.loadPoints( cloud );
    
    SQ_params par_in, par_out;
    par_in = par;
    par_in.a += 0.01;
    par_in.c -= 0.02;
    par_in.py += 0.03;
    par_in.e1 = 0.8;
    par_in.e2 = 0.8;

    if( mM.minimize2(par_in, par_out) ) {
	std::cout << "\t SUCCESS!"<< std::endl;
    } else {
	std::cout << "\t FAILURE!"<< std::endl;
    }
    
    // 3. Debug: Visualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0, 0);
    viewer->initCameraParameters ();

    // 3.1. Add initial pointcloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color( cloud, 0, 0, 255 );
    viewer->addPointCloud( cloud, color, "initialPCD");

    // 3.2. Add initial SQ guess
    visualizeSQ( viewer, 
		 par_in,
		 std::string("initial_guess"),
		 255, 0, 0 );

    // 3.3. Add final fitting SQ result
    visualizeSQ( viewer, 
		 par_out,
		 std::string("final_fit"),
		 0, 255, 0 );

    while (!viewer->wasStopped ()) {
	viewer->spinOnce (100);
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }    


    return 0;
}
