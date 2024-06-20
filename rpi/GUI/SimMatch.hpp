
#ifndef P7_SIMMATCH_HPP_
#define P7_SIMMATCH_HPP_

// Returns a time match for the simulation given
// the angle and the angular velocity of the pendulum
// as well as a first hint for the search
double simMatch(double angle, double angular_vel, double time_hint, double total_period, bool& success);

#endif