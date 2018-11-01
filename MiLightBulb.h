#pragma once

#include <RF24.h>
#include <openmili.h>

namespace mys_toolkit {

class CctMiLightBulb
{
  MiLightRadio _radio;
  uint16_t _deviceId;
  uint8_t _groupId;
  uint8_t _repetitions;
  uint8_t _sequenceId = 0;
  
  void _sendCommand(uint8_t command);
public:
  CctMiLightBulb(AbstractPL1167 &pl1167, uint16_t deviceId, uint8_t groupId, uint8_t repetitions=20);
  int begin();
  void setOn();
  void setOff();
  void brightnessUp();
  void brightnessDown();
  void temperatureUp();
  void temperatureDown();
  void pair();
  void unpair();
};

} //mys_toolkit
