#pragma once

#include <MySensorsToolkit/Actuator/Dimmer.h>
#include "MiLightBulb.h"

namespace mys_toolkit {

class MiLightDimmerDriver : public DimmerDriver
{
  CctMiLightBulb bulb_;
  uint8_t currentLevel_ = 0;

public:
  MiLightDimmerDriver(AbstractPL1167 &pl1167, uint16_t deviceId, uint8_t groupId, bool inverted);
  void setLevel(uint8_t level) override;
  void begin() override;
};

} //mys_toolkit
