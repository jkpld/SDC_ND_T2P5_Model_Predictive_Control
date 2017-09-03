#include "Poly.h"

// Evaluate the k'th derivative of a polynomial with coefficients coeffs at
// position x.
double polyeval(int k, Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = k; i < coeffs.size(); i++) {
    double mult = 1;
    for (int j=0; j < k; j++) mult *= i-j;
    result += mult * coeffs[i] * pow(x, i-k);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}