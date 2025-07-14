#include "APDS9930Switch.h"

#ifdef MYS_TOOLKIT_DEBUG
extern HardwareSerial MYS_TOOLKIT_SERIAL;
#endif

namespace mys_toolkit {

void MyAPDS9930::pcaSelect_(uint8_t i)
{
  if (apdsNum_ == 1)
    return;
  else {
    if (i > apdsNum_)
      return;
    Wire.beginTransmission(PCAADDR);
    Wire.write(4 + i);
    Wire.endTransmission();
  }
}

uint8_t MyAPDS9930::pcaGet_()
{
  if (apdsNum_ == 1)
    return digitalRead(intPin_) == LOW ? 1 : 0;
  else {
    Wire.requestFrom(PCAADDR, 1);
    return Wire.read() >> 4;
  }
}

void MyAPDS9930::init_(uint8_t i)
{
  pcaSelect_(i);
  apds_status_[i] = apds_[i].init();
  if (!apds_status_[i]) return;
  apds_status_[i] = apds_[i].enableProximitySensor(true);
  if (!apds_status_[i]) return;
  apds_status_[i] = apds_[i].setProximityGain(PGAIN_1X);
  if (!apds_status_[i]) return;
  apds_status_[i] = apds_[i].setProximityIntLowThreshold(PROX_INT_LOW);
  if (!apds_status_[i]) return;
  apds_status_[i] = apds_[i].setProximityIntHighThreshold(PROX_INT_HIGH);
  if (!apds_status_[i]) return;
}

bool MyAPDS9930::update_(uint8_t i)
{
  pcaSelect_(i);
  if (apds_[i].getProximityInt() == 0)
    return false;
  uint16_t proximity_data = 0;
  #ifdef MYS_TOOLKIT_DEBUG
  MYS_TOOLKIT_SERIAL.print("Reading sensor #");
  MYS_TOOLKIT_SERIAL.println(i);
  #endif
  if (!apds_[i].readProximity(proximity_data)) {
    #ifdef MYS_TOOLKIT_DEBUG
    MYS_TOOLKIT_SERIAL.println("Error reading proximity value");
    #endif
  } else {
    #ifdef MYS_TOOLKIT_DEBUG
    MYS_TOOLKIT_SERIAL.print("Proximity detected! Level: ");
    MYS_TOOLKIT_SERIAL.println(proximity_data);
    #endif
  }
  if (proximity_data < PROX_INT_HIGH) {
    if (!apds_[i].clearProximityInt()) {
      #ifdef MYS_TOOLKIT_DEBUG
      MYS_TOOLKIT_SERIAL.println("Error clearing interrupt");
      #endif
    }
  }
  return true;
}

MyAPDS9930::MyAPDS9930(uint8_t intPin, int apdsNum)
  : intPin_(intPin), apdsNum_(apdsNum), apdsInts_(0)
{
}

void MyAPDS9930::init()
{
  pinMode(intPin_, INPUT_PULLUP);
  for (uint8_t i=0; i<apdsNum_; i++)
    init_(i);
}

void MyAPDS9930::update()
{
  for (uint8_t i=0; i<apdsNum_; i++) {
    if (!apds_status_[i]) {
      init_(i);
    }
  }
  uint8_t pca = pcaGet_();
  apdsInts_ &= pca;
  if (pca) {
    for (uint8_t i=0; i<apdsNum_; i++) {
      if (pca & (1 << i)) {
        if (update_(i)) {
          apdsInts_ |= (1 << i);
        }
      }
    }
  }
}

bool MyAPDS9930::getInt(uint8_t i) const
{
  return apdsInts_ & (1 << i);
}

bool APDS9930Switch::doUpdate_()
{
  return myApds_.getInt(apdsNo_);
}

APDS9930Switch::APDS9930Switch(const MyAPDS9930 &myApds, uint8_t apdsNo)
  : Switch(false), myApds_(myApds), apdsNo_(apdsNo)
{
}

} //mys_toolkit
