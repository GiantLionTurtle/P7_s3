
#ifndef ARDUINO_MODEL_HPP_
#define ARDUINO_MODEL_HPP_

#include "../../common_rpiarduino/Common.hpp"

struct ArduinoModel {
  unsigned long long time_ms, simulation_start;
  double pendulum_angle;
  double pendulum_dangle;
  double wheelAngSpeed;
  double linSpeed;
  double wheel_pos;
  State state;

  void set_state(State state_)
  {
    state = state_;
    if(state == State::Swinging) {
      simulation_start = time_ms;
    }
  }
  void set_wheel_angSpeed(double wheelAngSpeed_)
  {
    wheelAngSpeed = wheelAngSpeed_;
    linSpeed = wheelAngSpeed * wheelRadius;
  }
};

#endif