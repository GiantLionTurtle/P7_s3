
#ifndef P7_POTWRAPPER_HPP_
#define P7_POTWRAPPER_HPP_

#include <Arduino.h>

// Light wrapper around a potentiometer
// used for the pendulum angle

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


struct PotWrapper {
  double minRange, maxRange;
  double zero;
  int curr, last;

  PotWrapper() = default;
  PotWrapper(double min_, double max_, int zero_)
    : minRange(min_)
    , maxRange(max_)
    , zero(map(static_cast<double>(zero_), 0.0, 1023.0, minRange, maxRange))
  {

  }

  void update(int val)
  {
    // Serial.println(val-zero);
    last = curr;
    curr = val;
  }
  int raw() const { return curr; }
  double position() const { return position(curr); }
  double position(int val) const { return map(static_cast<double>(val), 0.0, 1023.0, minRange, maxRange) - zero; }
  double speed() const { return position(curr) - position(last); }
};

#endif