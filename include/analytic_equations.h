/**
 * @file analytic_equations.h
 * @brief Analytic solutions to SQ equations (derived using MATLAB)
 */
#pragma once

#include "SQ_parameters.h"

double f_SQ( const SQ_parameters &_par, 
	     const double &_x, const double &_y, const double &_z );

void jac_SQ( const SQ_parameters &_par, 
	     const double &_x, const double &_y, const double &_z,
	     double _J[11] );

void hess_SQ( const SQ_parameters &_par, 
	      const double &_x, const double &_y, const double &_z,
	      double _H[][11] );

double error_SQ( const SQ_parameters &_par,  
		 const double &_x, const double &_y, const double &_z ); 
