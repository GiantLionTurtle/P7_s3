
#ifndef P7_TICKSWRAPPER_HPP_
#define P7_TICKSWRAPPER_HPP_

#include <Arduino.h>

struct TicksWrapper {
  long double conversion_ratio { 1.0 };
  long double offset;

  long long curr { 0 }, last { 0 };
  long long last_speed { 0 };

  double position_ { 0.0 };
  double speed_ { 0.0 };
  double accel_ { 0.0 };

  TicksWrapper() = default;
  TicksWrapper(double conv_ratio, double offset_ = 0.0) 
    : conversion_ratio(conv_ratio)
    , offset(offset_)
  {

  }

  void update(long long ticks_, double dt)
  {
    last_speed = dticks();
    last = curr; 
    curr = ticks_;

    double old_position = position_;
    double old_speed = speed_;
    position_ = static_cast<double>(ticks())  * conversion_ratio + offset;
    speed_ = (position_ - old_position) / dt;
    accel_ = (speed_ - old_speed) / dt;
  }

  long long ticks() const { return curr; }
  long long dticks() const { return curr-last; }
  long long ddticks() const { return dticks() - last_speed; }

  double position() const { return position_; }
  double speed() const { return speed_; }
  double accel() const { return accel_; }
};

#endif