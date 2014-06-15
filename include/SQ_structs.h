/**
 * @file SQ_structs.h
 */
#pragma once

/**
 * @struct SQBaseEquation
 * @brief Residual to be minimized
 */
struct SQBaseEquation {

  SQBaseEquation( double _x, double _y, double _z ) : 
    x_(_x), y_(_y), z_(_z) {}
  template<typename T> bool operator() ( const T* const dim, // 3
 					 const T* const e, // 2
					 const T* const trans, // 3
					 const T* const rot, // 3
					 T* residual ) const {
    
    // Get points in SQ frame
    T xe; T ye; T ze;

    T nx = cos( rot[2] )*cos( rot[1] );
    T ny = sin(rot[2])*cos(rot[1]);
    T nz = -sin(rot[1]);
    
    T ox = cos(rot[2])*sin(rot[1])*sin(rot[0]) - sin(rot[2])*cos(rot[0]);
    T oy = sin(rot[2])*sin(rot[1])*sin(rot[0]) + cos(rot[2])*cos(rot[0]);
    T oz = cos(rot[1])*sin(rot[0]);
    
    T ax = cos(rot[2])*sin(rot[1])*cos(rot[0]) + sin(rot[2])*sin(rot[0]);
    T ay = sin(rot[2])*sin(rot[1])*cos(rot[0]) - cos(rot[2])*sin(rot[0]);
    T az = cos(rot[1])*cos(rot[0]);

    xe = nx*T(x_) + ny*T(y_) + nz*T(z_) - trans[0]*nx - trans[1]*ny - trans[2]*nz;
    ye = ox*T(x_) + oy*T(y_) + oz*T(z_) - trans[0]*ox - trans[1]*oy - trans[2]*oz;
    ze = ax*T(x_) + ay*T(y_) + az*T(z_) - trans[0]*ax - trans[1]*ay - trans[2]*az;

    T F; T fxy; T fz;

    fxy = pow( ceres::abs(xe/dim[0]), T(2.0)/e[1] ) + pow( ceres::abs(ye/dim[1]), T(2.0)/e[1] );
    fz =  pow( ceres::abs(ze/dim[2]), T(2.0)/e[0] );
    F = pow( ceres::abs(fxy), T(e[1]/e[0]) ) + fz;    
    residual[0] = sqrt(dim[0]*dim[1]*dim[2])*( pow(F, e[1]) -T(1.0) );
    return true;
    
  }

private:
  double x_;
  double y_;
  double z_;
};
