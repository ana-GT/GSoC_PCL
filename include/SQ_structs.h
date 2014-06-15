/**
 * @file SQ_structs.h
 */
#pragma once

/**
 * @struct SQBaseEquation
 * @brief
 */
struct SQBaseEquation {

  SQBaseEquation( double _x, double _y, double _z ) : 
    x_(_x), y_(_y), z_(_z) {}
  template<typename T> bool operator() ( const T* const dim,
					 const T* const e,
					 T* residual ) const {
    

    // F(x) : Base SQ equation
    T F; T fxy; T fz;
    fxy = pow( ceres::abs( T(x_)/dim[0]), T(2.0)/e[1] ) + pow( ceres::abs(T(y_)/dim[1]), T(2.0)/e[1] );
    fz =  pow( ceres::abs( T(z_)/dim[2]), T(2.0)/e[0] );
    F = pow( ceres::abs(fxy), T(e[1]/e[0]) ) + fz;    
    residual[0] = pow(F, e[1]) -T(1.0);
    return true;
    
  }

private:
  double x_;
  double y_;
  double z_;
};
