#ifndef Poly_H
#define Poly_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// Evaluate the k'th derivative of a polynomial with coefficients coeffs at
// position x.
double polyeval(int k, Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

#endif
