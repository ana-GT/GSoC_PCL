/**
 * @file SQ_test.cpp
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>

#include "SQ_sampler.h"
#include "SQ_params.h"
#include "SQ_utils.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    // Create the sampler class
    SQ_sampler sqs;


    // Sample SQ naive
    SQ_params par; 
    par.a = 1.5; par.b = 1.0; par.c = 2.75;
    par.e1 = 0.8; par.e2 = 0.8;
    
    Eigen::Isometry3d Tf = Eigen::Isometry3d::Identity();
    Tf.translation() = Eigen::Vector3d( 0.5, 0.5, 0.8 );
    Eigen::Matrix3d rot;
    rot << 1,0,0, 0,0.7, 0.7, 0,-0.7, 0.7; 
    Tf.linear() = rot;

    transf2Params( Tf, par );

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
    cloud = sqs.sampleSQ_naive( par );
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("3DViewer") );
    viewer->addCoordinateSystem(1.0, "main", 0);
    viewer->addPointCloud( cloud, "naive", 0 );
    while( !viewer->wasStopped() ) {
	viewer->spinOnce(100);
	boost::this_thread::sleep( boost::posix_time::microseconds(100000) );
    }
    
    std::cout <<"Finished SQ_test.cpp "<< std::endl;

    return 0;
}
