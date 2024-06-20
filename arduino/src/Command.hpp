
#ifndef P7_COMMAND_HPP_
#define P7_COMMAND_HPP_

#include "../../common_rpiarduino/Common.hpp"

struct Command {
  unsigned int startTime_ms;
  float Tm[N_ACCELS_SAMPLES];

  float get_torque(unsigned int currTime_ms) const;
};

#endif