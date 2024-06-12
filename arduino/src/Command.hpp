
#ifndef P7_COMMAND_HPP_
#define P7_COMMAND_HPP_

#define COMMAND_DURATION_MS 100
#define N_TORQUE_SAMPLES 10

struct Command {
  unsigned int startTime_ms;
  float Tm[N_TORQUE_SAMPLES];

  float get_torque(unsigned int currTime_ms) const;
};

#endif