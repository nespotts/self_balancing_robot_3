// Button on pin 31: press -> motors off, USB disconnect, restart the Teensy.
#pragma once
#include <Bounce2.h>

class RebootButton {
 public:
  void begin();
  void update();
  static void reboot();

 private:
  Bounce b_;
};
