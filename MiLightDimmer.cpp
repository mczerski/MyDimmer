#include "MiLightDimmer.h"

#ifdef MYS_TOOLKIT_DEBUG
extern HardwareSerial MYS_TOOLKIT_SERIAL;
#endif

namespace mys_toolkit {

void MiLightDimmer::setLevel_(uint8_t level)
{
  level = round(10.0*level/255);
  int8_t diff = level - currentLevel_;
  if (diff > 0) {
    for (size_t i=0; i<static_cast<size_t>(diff); i++) {
      if (currentLevel_ + i == 0) {
        bulb_.setOn();
      }
      else if (currentLevel_ + i < 5) {
        bulb_.brightnessUp();
      }
      else {
        bulb_.brightnessUp();
        bulb_.temperatureDown();
      }
    }
  }
  else if (diff < 0) {
    for (size_t i=0; i<static_cast<size_t>(-diff); i++) {
      if (currentLevel_ - i >= 5) {
        bulb_.temperatureUp();
        bulb_.brightnessDown();
      }
      else if (currentLevel_ - i > 1) {
        bulb_.brightnessDown();
      }
      else {
        bulb_.setOff();
      }
    }
  }
  currentLevel_ = level;
}

MiLightDimmer::MiLightDimmer(AbstractPL1167 &pl1167, uint16_t deviceId, uint8_t groupId,
                             bool inverted, uint8_t dimmSpeed, Functions functions)
  : Dimmer(inverted, dimmSpeed, functions),
    bulb_(pl1167, deviceId, groupId)
{
}

void MiLightDimmer::begin()
{
  if (bulb_.begin() != 0) {
    #ifdef MYS_TOOLKIT_DEBUG
    MYS_TOOLKIT_SERIAL.println("MiLightRelay init failed");
    #endif
  }
  for (size_t i=0; i<5; i++)
    bulb_.setOn();
  for (size_t i=0; i<10; i++) {
    bulb_.brightnessDown();
    bulb_.temperatureUp();
  }
  bulb_.setOff();
}

} //mys_toolkit
