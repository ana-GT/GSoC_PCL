/**
 * @file SQ_utils.h
 * @author A. Huaman Quispe <ahuaman3@gatech.edu>
 * @brief Utilities 
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/visualization/pcl_visualizer.h>

#include "SQ_parameters.h"

void printParamsInfo( const SQ_parameters &_par );

void visualizeSQ(  boost::shared_ptr<pcl::visualization::PCLVisualizer> &_viewer,
		   const SQ_parameters &_par,
		   std::string _name,
		   int _r, int _g, int _b );

pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSQ_naive( const SQ_parameters &_par );

pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSE_FranklinBarr( const double &_a1, 
							   const  double &_a2,
							   const double &_e,
							   const int &_N );

pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSQ_PiluFischer( const SQ_parameters &_par );
