// Enable debug prints to serial monitor
//#define MY_DEBUG
//#define MY_DEBUG_VERBOSE_RF24
//#define MY_DEBUG_VERBOSE_RFM69
//#define MY_MY_DEBUG

#define LIVINGROOM
#define SKETCH_NAME "Dimmer"
#define SKETCH_MAJOR_VER "2"
#define SKETCH_MINOR_VER "1"

#ifdef KITCHEN
#define MY_NODE_ID 8
#define TEMP_PIN A4
#endif

#ifdef LIVINGROOM
#define MY_NODE_ID 6
#define USE_APDS9930
#define APDS9930_INT A3
#define APDS9930_NUM 1
#define TEMP_PIN A1
#endif

#ifdef BATHROOM1
#define MY_NODE_ID 10
#define TEMP_PIN A4
#define APDS9930_NUM 1
#endif

#ifdef BATHROOM2
#define MY_NODE_ID 9
#define TEMP_PIN A4
#define APDS9930_NUM 1
#endif

#ifdef BEDROOM
#define MY_NODE_ID 11
#define USE_APDS9930
#define APDS9930_INT A3
#define APDS9930_NUM 2
#define TEMP_PIN A1
#endif

#ifdef LIVINGROOM_SCENE
#define MY_NODE_ID 12
#endif

// Enable and select radio type attached
//#define MY_RADIO_NRF24
#define MY_RF24_CE_PIN 7
#define MY_RF24_CS_PIN 4
#define MY_RF24_PA_LEVEL RF24_PA_MAX
#define MY_RF24_CHANNEL 100

#define MY_RADIO_RFM69
#define MY_RFM69_CS_PIN 4
#define MY_RFM69_NEW_DRIVER
#define MY_RFM69_ATC_MODE_DISABLED
#define MY_RFM69_TX_POWER_DBM 0
//#define MY_IS_RFM69HW

#define MY_OTA_FIRMWARE_FEATURE
#define MY_TRANSPORT_WAIT_READY_MS 1

#include <MySensors.h>
#include "MyMySensors/MyMySensorsBase.h"

using namespace mymysensors;

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = (TCCR0B & 0b11111000) | mode;
    } else {
      TCCR1B = (TCCR1B & 0b11111000) | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = (TCCR2B & 0b11111000) | mode;
  }
}

#ifdef USE_APDS9930
#include "MyMySensors/APDS9930Switch.h"
MyAPDS9930 myApds(APDS9930_INT);
#endif

#ifdef KITCHEN
#define DIMMER1
#define DIMMER2
#include "MyMySensors/BounceSwitch.h"
BounceSwitch sw1(A1, MyDuration(50), true);
BounceSwitch sw2(A2, MyDuration(50), true);
BounceSwitch sw3(A3, MyDuration(50), true);
#include "MyMySensors/Dimmer.h"
CwWwDimmer dim1(9, 10, false, 10, {1, 1});
SimpleDimmer dim2(3, false, 10, {1, 1});
#include "MyMySensors/MyRelaySwitch.h"
MyRelaySwitch relay3(2, sw3, 5);
#endif

#ifdef LIVINGROOM
#define DIMMER1
#define DIMMER2
#define DIMMER3
#include "MyMySensors/BounceSwitch.h"
BounceSwitch sw1(A2, MyDuration(50), true);
BounceSwitch sw2(A2, MyDuration(50), true);
BounceSwitch sw3(APDS9930_INT, MyDuration(50), true);
#include "MyMySensors/Dimmer.h"
CwWwDimmer dim1(3, 5, true, 10, {1, 1});
CwWwDimmer dim2(9, 10, true, 10, {1, 1});
SimpleDimmer dim3(6, true, 10, {0, 0});
#endif

#ifdef BATHROOM1
#define DIMMER1
#define SCENE2
#define SCENE2_ENABLE_SHORT true
#define SCENE3
#define SCENE3_ENABLE_SHORT true
#include "MyMySensors/BounceSwitch.h"
BounceSwitch sw1(A3, MyDuration(50), true);
BounceSwitch sw2(A2, MyDuration(50), true);
BounceSwitch sw3(A1, MyDuration(50), true);
#include "MyMySensors/Dimmer.h"
SimpleDimmer dim1(10, false, 10, {1, 1});
#endif

#ifdef BATHROOM2
#define DIMMER1
#define SCENE2
#define SCENE2_ENABLE_SHORT true
#define SCENE3
#define SCENE3_ENABLE_SHORT true
#include "MyMySensors/BounceSwitch.h"
BounceSwitch sw1(A3, MyDuration(50), true);
BounceSwitch sw2(A2, MyDuration(50), true);
BounceSwitch sw3(A1, MyDuration(50), true);
#include "MyMySensors/Dimmer.h"
SimpleDimmer dim1(10, false, 10, {1, 1});
#endif

#ifdef BEDROOM
#define DIMMER1
#define DIMMER2
#define SCENE1
#define SCENE1_ENABLE_SHORT false
#define SCENE2
#define SCENE2_ENABLE_SHORT false
APDS9930Switch sw1(myApds, 0);
APDS9930Switch sw2(myApds, 1);
#include "MyMySensors/Dimmer.h"
SimpleDimmer dim1(3, true, 10, {0, 0});
SimpleDimmer dim2(5, true, 10, {0, 0});
#endif

#ifdef LIVINGROOM_SCENE
#define SCENE1
#define SCENE1_ENABLE_SHORT true
#define SCENE2
#define SCENE2_ENABLE_SHORT true
#define SCENE3
#define SCENE3_ENABLE_SHORT true
#define SCENE4
#define SCENE4_ENABLE_SHORT true
#include "MyMySensors/BounceSwitch.h"
BounceSwitch sw1(5, MyDuration(50), true);
BounceSwitch sw2(6, MyDuration(50), true);
BounceSwitch sw3(A5, MyDuration(50), true);
BounceSwitch sw4(A4, MyDuration(50), true);
#endif

#ifdef DIMMER1
#include "MyMySensors/MyDimmerSwitch.h"
MyDimmerSwitch dimmer1(0, dim1, sw1);
#endif
#ifdef DIMMER2
#include "MyMySensors/MyDimmerSwitch.h"
MyDimmerSwitch dimmer2(1, dim2, sw2);
#endif
#ifdef DIMMER3
#include "MyMySensors/MyDimmerSwitch.h"
MyDimmerSwitch dimmer3(2, dim3, sw3);
#endif
#ifdef SCENE1
#include "MyMySensors/MySceneController.h"
MySceneController scene1(3, sw1, SCENE1_ENABLE_SHORT);
#endif
#ifdef SCENE2
#include "MyMySensors/MySceneController.h"
MySceneController scene2(4, sw2, SCENE2_ENABLE_SHORT);
#endif
#ifdef SCENE3
#include "MyMySensors/MySceneController.h"
MySceneController scene3(5, sw3, SCENE3_ENABLE_SHORT);
#endif
#ifdef SCENE4
#include "MyMySensors/MySceneController.h"
MySceneController scene4(7, sw4, SCENE4_ENABLE_SHORT);
#endif

#ifdef TEMP_PIN
#include "MyMySensors/MyTemperatureSensor.h"
MyTemperatureSensor tempSensor(TEMP_PIN, 6, MyDuration(60000));
#endif

/***
 * Dimmable LED initialization method
 */
void setup()
{
  //MyDuration::setClockPrescaler(CLOCK_PRESCALER_2);
  setPwmFrequency(3, 64); //244Hz, also pin 11
  setPwmFrequency(5, 64); //488Hz, also pin 6
  setPwmFrequency(9, 64); //244Hz, also pin 10
  Serial.begin(115200);
  MyMySensorsBase::begin();
  #ifdef USE_APDS9930
  myApds.init();
  #endif
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

  // Register the LED Dimmable Light with the gateway
  MyMySensorsBase::present();
}

/***
 *  Dimmable LED main processing loop 
 */
void loop()
{
  #ifdef USE_APDS9930
  myApds.update();
  #endif
  MyMySensorsBase::update();
}

void receive(const MyMessage &message) {
  MyMySensorsBase::receive(message);
}

