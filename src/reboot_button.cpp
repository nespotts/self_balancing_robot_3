#include "reboot_button.h"

#include <Arduino.h>

#include "config.h"
#include "motors.h"

void RebootButton::begin() {
  b_.attach(pins::REBOOT_BTN, INPUT);
  b_.interval(5);
}

void RebootButton::update() {
  b_.update();
  if (b_.fell()) reboot();
}

void RebootButton::reboot() {
  motors_stop();
  USB1_USBCMD = 0;         // disconnect USB so the host notices
  delay(50);
  SCB_AIRCR = 0x05FA0004;  // restart
}
