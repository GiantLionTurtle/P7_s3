#ifndef PINCE_HPP_
#define PINCE_HPP_

#include <Arduino.h>
#include <LibS3GRO.h>

class Pince {
private :
  bool open = true;
  MegaServo megaservo_;
public:
  void Init(int pin) { megaservo_.attach(pin); }
  void Open() { megaservo_.write(0); open = true; }
  void close() { megaservo_.write(60); open = false; }
  bool isOpen() { return open; }
};

#endif