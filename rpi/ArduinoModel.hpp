
#ifndef ARDUINO_MODEL_HPP_
#define ARDUINO_MODEL_HPP_

#include "../../common_rpiarduino/Common.hpp"

struct ArduinoModel {
  unsigned long int time_ms;
  double pendulum_angle;
  double pendulum_dangle;
  double wheelAngSpeed;
  double linSpeed;
  double wheel_pos;
  State state;
};

#endif