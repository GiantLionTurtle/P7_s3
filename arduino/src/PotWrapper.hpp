
#ifndef P7_POTWRAPPER_HPP_
#define P7_POTWRAPPER_HPP_

#include <Arduino.h>

// Light wrapper around a potentiometer
// used for the pendulum angle
struct PotWrapper {
  double minRange, maxRange;
  int curr, last;

  PotWrapper() = default;
  PotWrapper(double min_, double max_)
    : minRange(min_)
    , maxRange(max_)
  {

  }

  void update(int val)
  {
    last = curr;
    curr = val;
  }
  int raw() const { return curr; }
  double position() const { return position(curr); }
  double position(int val) const { return map(static_cast<double>(val), 0.0, 1023.0, minRange, maxRange); }
  double speed() const { return position(curr) - position(last); }
};

#endif