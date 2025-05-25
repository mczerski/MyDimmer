#ifndef PWMSwitch_h
#define PWMSwitch_h

#include <MySensorsToolkit/Actuator/Switch.h>

namespace mys_toolkit {

class PWMSwitch : public Switch {
  int pin_;
  bool activeLow_;
  int prev_state_ = 0;
  bool doUpdate_() override;

public:
  PWMSwitch(uint8_t pin, bool activeLow);
};

} //mys_toolkit

#endif //PWMSwitch_h
