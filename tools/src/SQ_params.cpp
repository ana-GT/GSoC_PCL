/**
 * @file SQ_params.cpp
 * @author A.C.H.Q. <ahuaman3@gatech.edu>
 * @date 2014/06/02
 * @brief 
 */
#include "SQ_params.h"

/**
 * @function SQ_params
 * @brief Constructor Initialize default values (probably inappropriate)
 */
SQ_params::SQ_params() {

    a1_ = 0.5;
    a2_ = 0.5;
    a3_ = 0.5;

    e1_ = 0.5;
    e2_ = 0.5;
}

/**
 * @function SQ_params
 * @brief Constructors with input parameters
 */
SQ_params::SQ_params( double _a1, double _a2, double _a3,
		      double _e1, double _e2 ) {

    a1_ = _a1;
    a2_ = _a2;
    a3_ = _a3;
    e1_ = _e1;
    e2_ = _e2;
}


SQ_params::~SQ_params() {
}

bool SQ_params::set( double _a1, double _a2, double _a3,
		     double _e1, double _e2 ) {

    a1_ = _a1;
    a2_ = _a2;
    a3_ = _a3;
    e1_ = _e1;
    e2_ = _e2;
}
