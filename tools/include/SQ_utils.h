/**
 * @file SQ_utils.h
 * @author A.C.H.Q. <ahuaman3@gatech.edu>
 * @brief Utilities 
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "SQ_params.h"

Eigen::Isometry3d param2Transf( const SQ_params &_par );

void transf2Params( const Eigen::Isometry3d &_Tf,
		    SQ_params &_par );

Eigen::VectorXd params2Vec( const SQ_params &_par );
bool vec2Param( const Eigen::VectorXd &_vec,
		SQ_params &_par );

void printParamsInfo( const SQ_params &_par );
