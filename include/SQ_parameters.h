/**
 * @function SQ_parameters.h
 */
#pragma once


/**
 * @struct SQ_parameters
 */
struct SQ_parameters {

  /** Dimensions: a,b,c */
  double dim[3];

  /** Coefficients: e1 & e2 */
  double e[2];

  /** Translation & Rotation */
  double trans[3];
  double rot[3];

};
