/**
 * @file SQ_utils.h
 * @author A.C.H.Q. <ahuaman3@gatech.edu>
 * @brief Utilities 
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/visualization/pcl_visualizer.h>

#include "SQ_params.h"

Eigen::Isometry3d param2Transf( const SQ_params &_par );

void transf2Params( const Eigen::Isometry3d &_Tf,
		    SQ_params &_par );

Eigen::VectorXd params2Vec( const SQ_params &_par );
bool vec2Param( const Eigen::VectorXd &_vec,
		SQ_params &_par );

void printParamsInfo( const SQ_params &_par );

void visualizeSQ(  boost::shared_ptr<pcl::visualization::PCLVisualizer> &_viewer,
		   const SQ_params &_par,
		   std::string _name,
		   int _r, int _g, int _b );
bool clamp( SQ_params &_par );
bool clamp( Eigen::VectorXd &_par );
