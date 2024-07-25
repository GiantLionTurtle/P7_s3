
#ifndef P7_TICKSWRAPPER_HPP_
#define P7_TICKSWRAPPER_HPP_

#include <Arduino.h>

struct TicksWrapper {
  long double conversion_ratio { 1.0 };
  long double offset;

  long long curr { 0 }, last { 0 };
  long long last_speed { 0 };

  TicksWrapper() = default;
  TicksWrapper(double conv_ratio, double offset_ = 0.0) 
    : conversion_ratio(conv_ratio)
    , offset(offset_)
  {

  }

  void update(long long ticks)
  {
    last_speed = dticks();
    last = curr; 
    curr = ticks;
  }

  long long ticks() const { return curr; }
  long long dticks() const { return curr-last; }
  long long ddticks() const { return dticks() - last_speed; }

  double position() const { return static_cast<double>(ticks()) * conversion_ratio + offset; }
  double speed() const { return static_cast<double>(dticks())   * conversion_ratio; }
  double accel() const { return static_cast<double>(ddticks())  * conversion_ratio; }
};

#endif