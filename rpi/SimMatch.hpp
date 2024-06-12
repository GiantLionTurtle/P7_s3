
#ifndef P7_SIMMATCH_HPP_
#define P7_SIMMATCH_HPP_

namespace p7 {

// Returns a time match for the simulation given
// the angle and the angular velocity of the pendulum
// as well as a first hint for the search
double simMatch(double angle, double angular_vel, double time_hint, bool& success);

} // !p7

#endif