#ifndef APDS9930Switch_h
#define APDS9930Switch_h

#include <MySensorsToolkit/Actuator/Switch.h>

#include <APDS9930.h>
#include <Wire.h>

namespace mys_toolkit {

class MyAPDS9930 {
  static constexpr int MAX_APDS9930_NUM = 4;
  uint8_t intPin_;
  int apdsNum_ = 0;
  APDS9930 apds_[MAX_APDS9930_NUM];
  uint8_t apdsInts_;
  bool apds_status_[MAX_APDS9930_NUM];
  static const int PCAADDR = 0x70;
  static const int PROX_INT_HIGH = 900; // Proximity level for interrupt
  static const int PROX_INT_LOW = 0;  // No far interrupt

  void pcaSelect_(uint8_t i);
  uint8_t pcaGet_();
  void init_(uint8_t i);
  bool update_(uint8_t i);

public:
  MyAPDS9930(uint8_t intPin, int apdsNum);
  void init();
  void update();
  bool getInt(uint8_t i) const;
};

class APDS9930Switch : public Switch {
  const MyAPDS9930 &myApds_;
  uint8_t apdsNo_;
  bool doUpdate_() override;
public:
  APDS9930Switch(const MyAPDS9930 &myApds, uint8_t apdsNo);
};

} //mys_toolkit

#endif //APDS9930Switch_h
