
#include "SimMatch.hpp"

#include <iostream>
#include <numbers>
#include <cmath>

#define AMP 0.366519142919
#define TT 2.0
#define COEFF 1.25
#define ANGVEL_REL_ERROR 0.3 // %
#define TRY_HINTDELTA 0.05     // s
#define LM_EPS1 1e-12
#define LM_EPS2 1e-12
#define LM_EPS3 1e-24

// Driving function of the simulation
// we want the effector angle to follow 
// this function
double sim_func(double t)
{
  double pi = 3.1415;
  return AMP/TT * t * std::sin(2*pi/TT*COEFF*t);
}
// Derivative of previous function 
double sim_deri(double t)
{
  double pi = 3.1415;
  return AMP/(TT*TT) * (TT*sin(2*pi*COEFF*t/TT) + 2*pi*COEFF*t*cos(2*pi*COEFF*t/TT));
}

// LM solver to find the closest zero to the hint time at an offset
double solve(double hint, double zero_off, double eps1 = LM_EPS1, double eps2 = LM_EPS2, double eps3 = LM_EPS3)
{
  // http://users.ics.forth.gr/~lourakis/levmar/levmar.pdf
  double v = 2.0;
  double p = hint;

  double J = sim_deri(p);
  double A = J*J;
  double Ep = zero_off - sim_func(p);
  double g = J * Ep;

  bool stop = std::abs(g) <= eps1;
  double mu = 1e-3*A;
  int max_it = 50;

  for(size_t k = 0; k < max_it && !stop; ++k) {
    double rho = 0.0;
    while(rho <= 0 && !stop) {
      double dp = g / (A+mu);
      if(std::abs(dp) <= eps2 * std::abs(p)) {
        stop = true;
      } else {
        double p_new = p + dp;
        double Ep_new = (zero_off-sim_func(p_new));
        rho = (Ep*Ep - Ep_new*Ep_new) / (dp*(mu*dp+g));

        if(rho > 0) {
          p = p_new;
          J = sim_deri(p);
          A = J*J;
          Ep = Ep_new;
          g = J*Ep;

          stop = std::abs(g) <= eps1 || Ep*Ep <= eps3;
          mu = mu * std::max(0.3333, 1.0-std::pow(2*rho-1, 3));
          v = 2.0;
        } else {
          mu *= v;
          v *= 2.0;
        }
      }
    }
  }
  return p;
}

// Finds the relative error between two values
double rel_error(double attempt, double target)
{
  double abs_error = attempt - target;
  double rel_error = std::abs(abs_error/target);

  return rel_error;
}
bool match_ok(double match, double angle, double angvel)
{
  return (rel_error(sim_func(match), angle) <= 0.01 && rel_error(sim_deri(match), angvel) <= ANGVEL_REL_ERROR);
}

// Tries to find the closest point on the simulation path to the actual angle/angular velocity pair
// by solving repeatedly with time hints that go further and further away
// from the initial time hint
double simMatch(double angle, double angular_vel, double time_hint, double total_period, bool& success)
{
  double match = solve(time_hint, angle);

  if(match_ok(match, angle, angular_vel)) {
    success = true;
    return match;
  }
  
  double future_hint = time_hint + TRY_HINTDELTA, past_hint = time_hint - TRY_HINTDELTA;
  while(future_hint < total_period && past_hint >= 0.0) {
    match = solve(future_hint, angle);
    if(match_ok(match, angle, angular_vel)) {
      success = true;
      return match;
    }
    match = solve(past_hint, angle);
    if(match_ok(match, angle, angular_vel)) {
      success = true;
      return match;
    }
    future_hint += TRY_HINTDELTA;
    past_hint -= TRY_HINTDELTA;
  }
  success = false;
  return match;
}