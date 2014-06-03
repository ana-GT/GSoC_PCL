/**
 * @file matlab_equations.cpp
 */
#pragma once

void jac_MATLAB( double a, double b, double c, double e1, double e2, double px, double py, double pz, double ra, double pa, double ya, double x, double y, double z, double Jac[] );

void hess_MATLAB( double a, double b, double c, double e1, double e2, double px, double py, double pz, double ra, double pa, double ya, double x, double y, double z, double Hess[][11] ); 
