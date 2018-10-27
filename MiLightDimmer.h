#pragma once

#include <MySensorsToolkit/Actuator/Dimmer.h>
#include "MiLightBulb.h"

namespace mys_toolkit {

class MiLightDimmer : public Dimmer
{
  CctMiLightBulb bulb_;
  uint8_t currentLevel_ = 0;
  void setLevel_(uint8_t level) override;

public:
  MiLightDimmer(AbstractPL1167 &pl1167, uint16_t deviceId, uint8_t groupId,
                bool inverted, uint8_t dimmSpeed, Functions functions);
  void begin() override;
};

} //mys_toolkit

