#include "MiLightBulb.h"

namespace mys_toolkit {

void CctMiLightBulb::_sendCommand(uint8_t command)
{
  uint8_t packet[7] = {0x5A, uint8_t(_deviceId >> 8), uint8_t(_deviceId), _groupId,
                       command, _sequenceId++, 0};
  uint8_t checksum = sizeof(packet);
  for (size_t i=0; i<sizeof(packet)-1; i++)
      checksum += packet[i];
  packet[sizeof(packet)-1] = checksum;
  _radio.write(packet, sizeof(packet));
  for (size_t i=1; i<_repetitions; i++) {
    _radio.resend();
  }
}

CctMiLightBulb::CctMiLightBulb(AbstractPL1167 &pl1167, uint16_t deviceId, uint8_t groupId, uint8_t repetitions)
  :  _radio(pl1167, {4, 39, 74, 0x050A, 0x55AA}), _deviceId(deviceId), _groupId(groupId), _repetitions(repetitions)
{}

int CctMiLightBulb::begin()
{
  return _radio.begin();
}

void CctMiLightBulb::setOn()
{
  uint8_t command = 0;
  switch (_groupId) {
    case 1:
      command = 0x08; break;
    case 2:
      command = 0x0D; break;
    case 3:
      command = 0x07; break;
    case 4:
      command = 0x02; break;
  }
  _sendCommand(command);
}

void CctMiLightBulb::setOff()
{
  uint8_t command = 0;
  switch (_groupId) {
    case 1:
      command = 0x0B; break;
    case 2:
      command = 0x03; break;
    case 3:
      command = 0x0A; break;
    case 4:
      command = 0x06; break;
  }
  _sendCommand(command);
}

void CctMiLightBulb::brightnessUp()
{
  _sendCommand(0x0C);
}

void CctMiLightBulb::brightnessDown()
{
  _sendCommand(0x04);
}

void CctMiLightBulb::temperatureUp()
{
  _sendCommand(0x0E);
}

void CctMiLightBulb::temperatureDown()
{
  _sendCommand(0x0F);
}

void CctMiLightBulb::pair() {
  setOn();
}

void CctMiLightBulb::unpair() {
  for (int i=0; i<5; i++)
    setOn();
}

} //mys_toolkit
