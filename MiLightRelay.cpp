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
  //updateState_(getState());
}

} //mys_toolkit
