#include "MPC.h"
#include "Poly.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the number of time steps
int N = 10;

// Number of state and actuator variables
const int N_state = 6;
const int N_actuator = 2;

// Start index for the i'th element of the variable array. State variables have
// N entries (starting at t=0) and actuator variabels have N-1 entries
// (starting at t=1)
int sIdx(int i) {
  return i*N + ((i<=N_state) ? 0 : -1*(i-N_state));
}

// helper function
inline AD<double> square(AD<double> x) {return CppAD::pow(x,2);}

// Distance from the front of the car to the center of gravity
const double Lf = 2.67;

/* FG_eval
  F : function to minimize
  G : constraint functions
  FG_eval : class to evaluate F and G. output are returned in vector fg
    where fg[0] = F, and fg[1:end] = G;
*/
class FG_eval {
 public:
  // Fitted polynomial coefficients
  // Eigen::VectorXd coeffs;
  double time_step;
  double latency;
  double kappa;
  double RoC;
  CPPAD_TESTVECTOR(AD<double>) coeffs;

  FG_eval(Eigen::VectorXd coeffs_in, double dt, double latency, double kappa, double RoC) {
    coeffs.resize(coeffs_in.size());
    for (int i=0; i<coeffs_in.size(); i++) {
      coeffs[i] = coeffs_in[i];
    }

    time_step = dt;
    this->latency = latency;
    this->kappa = kappa;
    this->RoC = RoC;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // VARS = ( x | y | psi | v | cte | epsi | delta | a )
    // vars ~ VARS(:);

    /* ==========================================================
      Function to minimize
    ========================================================== */
    // change the speed reference depending on the curvature
    double ref_v = 20 + 170*RoC;

    // weights for each variable cost
    double w_cte = 20;
    double w_psi = 20;
    double w_v = 30;
    double w_th = 0;
    double w_a = 0;
    double w_dth = 2 + 80*sqrt(RoC); //2 + 100*sqrt(RoC);
    double w_da = 0;

    // std::cout << ref_v << "\t" << time_step << "\t" << RoC << "\t";

    fg[0] = 0;

    // Minimize direct errors
    for (int t=0; t<N; t++) {
      // bias the cross-track error to be more significant at later points.
      fg[0] += (t+1) * w_cte * square(vars[sIdx(4)+t]); // cross track error, w_cte=20
      // fg[0] += w_cte * square(vars[sIdx(4)+t]); // cross track error, w_cte=120
      fg[0] += w_psi * square(57*vars[sIdx(5)+t]); // heading error
      fg[0] += w_v * square(vars[sIdx(3)+t] - ref_v); // speed error
    }

    // Minimize actuator values
    for (int t=0; t<N-1; t++) {
      fg[0] += w_th * square(vars[sIdx(6)+t]); // steering angle
      fg[0] += w_a * square(vars[sIdx(7)+t]); // acceleration
    }

    // Minimize derivative of actuator values
    for (int t=0; t<N-2; t++) {
      // The time step is different for the first step and the remaining
      double dt = (t==0) ? latency : time_step;
      fg[0] += w_dth * square( 57*(vars[sIdx(6)+t+1] - vars[sIdx(6)+t]) / dt); // steering angle derivative
      fg[0] += w_da * square( (vars[sIdx(7)+t+1] - vars[sIdx(7)+t]) / dt); // acceleration derivative
    }

    /* ==========================================================
      Constraints
    ========================================================== */

    // Set t=0 constraints
    for (int i=0; i<N_state; i++)
      fg[1+sIdx(i)] = vars[sIdx(i)];

    // Set constraints for t>0
    for (int t = 1; t<N; t++) {

      // The time step is different for the first step and all remaining steps
      double dt = (t==1) ? latency : time_step;

      auto x1 = vars[sIdx(0) + t];
      auto x0 = vars[sIdx(0) + t - 1];
      auto y1 = vars[sIdx(1) + t];
      auto y0 = vars[sIdx(1) + t - 1];
      auto psi1 = vars[sIdx(2) + t];
      auto psi0 = vars[sIdx(2) + t - 1];
      auto v1 = vars[sIdx(3) + t];
      auto v0 = vars[sIdx(3) + t - 1];
      auto cte1 = vars[sIdx(4) + t];
      auto epsi1 = vars[sIdx(5) + t];
      auto epsi0 = vars[sIdx(5) + t - 1];
      auto delta0 = vars[sIdx(6)+t-1];
      auto a0 = vars[sIdx(7)+t-1];

      // Values we want
      auto f0 = CppAD::Poly(0, coeffs, x0);
      auto psides0 = CppAD::atan(CppAD::Poly(1, coeffs, x0));

      // Constraint equations
      fg[1+sIdx(0)+t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1+sIdx(1)+t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1+sIdx(2)+t] = psi1 - (psi0 + v0 * delta0 * dt / Lf);
      fg[1+sIdx(3)+t] = v1 - (v0 + a0*dt);
      fg[1+sIdx(4)+t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1+sIdx(5)+t] = epsi1 - ((psi0 - psides0) + v0 * delta0 * dt / Lf);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double latency) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Number of variables and number of constraints
  int n_vars = N_state*N + N_actuator*(N-1);
  int n_constraints = N_state*N;

  /* ==========================================================
    Road curvature
  ========================================================== */
  double df = polyeval(1, coeffs, state[0]+25);
  double ddf = polyeval(2, coeffs, state[0]+25);
  double kappa = ddf/pow((1 + df*df),1.5);

  // Radius of curvature
  double RoC = 1/fabs(kappa);
  RoC = ((RoC > 500) ? 500 : RoC)/500;

  /* Compute the time step for modeling.
    Model a shorter distance if the radius of curvature is small and a longer
    distance if the radius of curvature is long.
  */
  double lookahead = 50 + 30*RoC - latency*state[3];
  double dt = lookahead/((N-1)*state[3] + 0.2);
  if (dt < 0.0001) dt = 0.0001;
  if (dt > 1) dt = 1;

  /* ==========================================================
    Initialize variables
  ========================================================== */
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
    vars[i] = 0;

  for (int i=0; i < N_state; i++)
    vars[sIdx(i)] = state[i];

  /* ==========================================================
    Set variable bounds
  ========================================================== */
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // state bounds are -inf, inf
  for (int i=0; i < sIdx(N_state); i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // steering bounds
  for (int i=sIdx(6); i < sIdx(7); i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // acceleration bounds
  for (int i=sIdx(7); i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  /* ==========================================================
    Set constraint bounds
  ========================================================== */
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  // set all to 0
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // set t=0 constraints are the state values
  for (int i=0; i < N_state; i++) {
    constraints_lowerbound[sIdx(i)] = state[i];
    constraints_upperbound[sIdx(i)] = state[i];
  }

  /* ==========================================================
    IPOPT solver
  ========================================================== */
  // Create solver object
  FG_eval fg_eval(coeffs,dt,latency,kappa,RoC);

  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // Initialize solution storage
  CppAD::ipopt::solve_result<Dvector> solution;

  // Solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  // auto cost = solution.obj_value;

  // std::cout << ok << "\t" << cost << std::endl;

  vector<double> result(2*(N+1));
  result[0] = solution.x[sIdx(6)]; // steering angle
  result[1] = solution.x[sIdx(7)]; // acceleration
  for (int i=0; i<N; i++) {
    result[2*(i+1)] = solution.x[sIdx(0)+i]; // save x_i
    result[2*(i+1)+1] = solution.x[sIdx(1)+i]; // save y_i
  }

  return result;
}
