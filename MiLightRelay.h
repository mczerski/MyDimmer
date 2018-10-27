#pragma once

#include <MySensorsToolkit/Actuator/Relay.h>
#include "MiLightBulb.h"

namespace mys_toolkit {

class MiLightRelay : public Relay
{
  CctMiLightBulb bulb_;
  void updateState_(bool state) override;

public:
  MiLightRelay(AbstractPL1167 &pl1167, uint16_t deviceId, uint8_t groupId);
  int begin() {return bulb_.begin();}
};

} //mys_toolkit
