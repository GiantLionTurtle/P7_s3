
#ifndef P7_POTWRAPPER_HPP_
#define P7_POTWRAPPER_HPP_

#include <Arduino.h>

// Light wrapper around a potentiometer
// used for the pendulum angle
struct PotWrapper {
  double minRange, maxRange;
  int zero;
  int curr;

  double position_ { 0.0 };
  double speed_ { 0.0 };

  PotWrapper() = default;
  PotWrapper(double min_, double max_)
    : minRange(min_)
    , maxRange(max_)
  {

  }

  void update(int val, double dt)
  {
    curr = val;

    double old_position = position_;
    position_ = position(curr);
    speed_    = (old_position - position_) / dt;
  }

  void calibrate(int val)
  {
    zero = map_me_up(val);
  }

  int raw() const { return curr; }
  double position() const { return position_; }
  double position(int val) const { return map_me_up(val) - zero; }
  double speed() const { return speed_; }
  double map_me_up(int val) const { return (static_cast<double>(val)) * (maxRange - minRange) / (1023.0) + minRange; }
};

#endif