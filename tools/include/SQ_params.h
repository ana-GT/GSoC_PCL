/**
 * @file SQ_params.h
 */
#pragma once

/**
 * @struct SQ_params
 */
struct SQ_params {
    
SQ_params() : a(0.5), b(0.5), c(0.5), e1(0.5), e2(0.5) {}
    
    double a, b, c;
    double e1, e2;
    double px, py, pz; /** Translation */
    double ra, pa, ya; /** roll, pitch, yaw */    
};

