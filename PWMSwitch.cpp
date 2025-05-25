#include "PWMSwitch.h"
#include <Arduino.h>

namespace mys_toolkit {

bool PWMSwitch::doUpdate_()
{
  int state = 0;
  unsigned long pulse_width = pulseIn(pin_, HIGH, 1000);
  if (pulse_width == 0) {
    if (digitalRead(pin_) == 1) {
      state = 3;
    }
  }
  else if (20 < pulse_width && pulse_width <= 80) {
    state = 1;
  }
  else if (80 < pulse_width && pulse_width < 300) {
    state = 2;
  }
  if (prev_state_ != state) {
    prev_state_ = state;
    return !activeLow_;
  }
  return activeLow_;
}

PWMSwitch::PWMSwitch(uint8_t pin, bool activeLow)
  : Switch(activeLow), pin_(pin), activeLow_(activeLow)
{
  pinMode(pin_, INPUT);
}

} //mys_toolkit
