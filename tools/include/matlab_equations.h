/**
 * @file matlab_equations.cpp
 */
#pragma once

#include <Eigen/Core>

Eigen::VectorXd jac_MATLAB( const Eigen::VectorXd &_params, 
			    double x, double y, double z );

void hess_MATLAB( const Eigen::VectorXd &_params, double x, double y, double z, double A0[][11] );

double error_MATLAB( const Eigen::VectorXd &_params, 
		     double x, double y, double z ); 
