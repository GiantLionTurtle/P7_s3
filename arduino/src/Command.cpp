

#include "Command.hpp"

#include <Arduino.h>

float Command::get_torque(unsigned int currTime_ms) const
{
  if(currTime_ms > startTime_ms + COMMAND_DURATION_MS) { // Invalid time stamp
    return 0.0f;
  }

  unsigned int rel_time_ms = currTime_ms - startTime_ms;

  float sample = static_cast<float>(rel_time_ms) / static_cast<float>(COMMAND_DURATION_MS / N_TORQUE_SAMPLES);
  unsigned int upper = ceil(sample);
  unsigned int lower = floor(sample);

  return Tm[lower] * (1.0-(sample-lower)) + Tm[upper] * (sample-lower);
}