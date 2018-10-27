#include "MiLightRelay.h"

namespace mys_toolkit {

void MiLightRelay::updateState_(bool state)
{
  if (state)
    bulb_.setOn();
  else
    bulb_.setOff();
}

MiLightRelay::MiLightRelay(AbstractPL1167 &pl1167, uint16_t deviceId, uint8_t groupId)
  : bulb_(pl1167, deviceId, groupId)
{
}

void MiLightRelay::begin()
{
  if (bulb_.begin() != 0)
    Serial.println("MiLightRelay init failed");
  for (size_t i=0; i<5; i++)
    updateState_(getState());
}

} //mys_toolkit
