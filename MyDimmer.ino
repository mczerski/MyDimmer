// Enable debug prints to serial monitor
//#define MY_DEBUG
//#define MY_DEBUG_VERBOSE_RF24
//#define MY_DEBUG_VERBOSE_RFM69
//#define MYS_TOOLKIT_DEBUG

#define KITCHEN
#define SKETCH_NAME "Dimmer"
#define SKETCH_MAJOR_VER "2"
#define SKETCH_MINOR_VER "6"

// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_RFM69_CS_PIN 4
#define MY_RFM69_NEW_DRIVER
#define MY_RFM69_ATC_MODE_DISABLED
#define MY_RFM69_TX_POWER_DBM 0
#define MY_OTA_FIRMWARE_FEATURE
#define MY_TRANSPORT_WAIT_READY_MS 1

#ifdef KITCHEN
#define MY_NODE_ID 8
#define SKETCH_SUBNAME "Kitchen"
#define TEMP_PIN A4
#endif

#ifdef LIVINGROOM
#define MY_NODE_ID 6
#define SKETCH_SUBNAME "Livingroom"
#define USE_APDS9930
#define APDS9930_INT A3
#define APDS9930_NUM 1
#define TEMP_PIN A1
#endif

#ifdef BATHROOM1
#define MY_NODE_ID 10
#define SKETCH_SUBNAME "LargeBathroom"
#define TEMP_PIN A4
#define APDS9930_NUM 1
#endif

#ifdef BATHROOM2
#define MY_NODE_ID 9
#define SKETCH_SUBNAME "SmallBathroom"
#define TEMP_PIN A4
#define APDS9930_NUM 1
#endif

#ifdef BEDROOM1
#define MY_NODE_ID 11
#define SKETCH_SUBNAME "LargeBedroom"
#define USE_APDS9930
#define APDS9930_INT A3
#define APDS9930_NUM 2
#define TEMP_PIN A1
#endif

#ifdef LIVINGROOM_SCENE
#define MY_NODE_ID 12
#undef MY_RFM69_CS_PIN
#define MY_RFM69_CS_PIN 10
#undef SKETCH_NAME
#define SKETCH_NAME "Scene"
#define SKETCH_SUBNAME "Livingroom"
#endif

#ifdef TEST
#define MY_NODE_ID 13
#undef MY_RFM69_CS_PIN
#define MY_RFM69_CS_PIN 10
#undef SKETCH_NAME
#define SKETCH_NAME "Test"
#define SKETCH_SUBNAME ""
#endif

#include <avr/wdt.h>
#include <MySensors.h>
#include <MySensorsToolkit.h>
#include "BounceSwitch.h"
#include "AnalogBounceSwitch.h"
#include "APDS9930Switch.h"
#include "DS18B20RequestableValue.h"
#include "MiLightRelay.h"
#include "MiLightDimmer.h"
#include <MySensorsToolkit/Actuator/DimmerActuator.h>
#include <MySensorsToolkit/Actuator/RelayActuator.h>
#include <MySensorsToolkit/Actuator/SceneController.h>

using namespace mys_toolkit;

#ifdef USE_APDS9930
MyAPDS9930 myApds(APDS9930_INT, APDS9930_NUM);
#endif

#ifdef KITCHEN
#define CLOCK_PRESCALER CLOCK_PRESCALER_2
#define DIMMER1
#define DIMMER2
#define DIMMER3
BounceSwitch sw1(A1, Duration(50), true);
BounceSwitch sw2(A2, Duration(50), true);
BounceSwitch sw3(A3, Duration(50), true);
CwWwDimmer dim1(9, 10, false, 10, {.slowDimming=1, .fullBrightness=1});
SimpleDimmer dim2(3, false, 10, {.slowDimming=1, .fullBrightness=1});
#define NRF24_CE_PIN 7
#define NRF24_CSN_PIN 6
RF24 nrf24Radio(NRF24_CE_PIN, NRF24_CSN_PIN);
PL1167_nRF24 pl1167(nrf24Radio);
MiLightDimmer dim3(pl1167, 0xF2EA, 4, false, 10, {.slowDimming=1, .fullBrightness=1});
#endif

#ifdef LIVINGROOM
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
#define DIMMER1
#define DIMMER3
BounceSwitch sw1(A2, Duration(50), true);
APDS9930Switch sw3(myApds, 0);
CwWwDimmerN<2> dim1({3, 9}, {5, 10}, true, 10, {.slowDimming=1, .fullBrightness=1});
SimpleDimmer dim3(6, true, 10, {.slowDimming=0, .fullBrightness=0});
#endif

#ifdef BATHROOM1
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
#define DIMMER1
#define SCENE2
#define SCENE2_ENABLE_SHORT true
#define SCENE3
#define SCENE3_ENABLE_SHORT true
BounceSwitch sw1(A3, Duration(50), true);
BounceSwitch sw2(A2, Duration(50), true);
BounceSwitch sw3(A1, Duration(50), true);
SimpleDimmer dim1(10, false, 10, {.slowDimming=1, .fullBrightness=1});
#endif

#ifdef BATHROOM2
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
#define DIMMER1
#define SCENE2
#define SCENE2_ENABLE_SHORT true
#define SCENE3
#define SCENE3_ENABLE_SHORT true
BounceSwitch sw1(A3, Duration(50), true);
BounceSwitch sw2(A2, Duration(50), true);
BounceSwitch sw3(A1, Duration(50), true);
SimpleDimmer dim1(10, false, 10, {.slowDimming=1, .fullBrightness=1});
#endif

#ifdef BEDROOM1
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
#define DIMMER1
#define DIMMER2
#define SCENE1
#define SCENE1_ENABLE_SHORT false
#define SCENE2
#define SCENE2_ENABLE_SHORT false
APDS9930Switch sw1(myApds, 0);
APDS9930Switch sw2(myApds, 1);
SimpleDimmer dim1(3, true, 10, {.slowDimming=0, .fullBrightness=0});
SimpleDimmer dim2(5, true, 10, {.slowDimming=0, .fullBrightness=0});
#endif

#ifdef LIVINGROOM_SCENE
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
#define SCENE1
#define SCENE1_ENABLE_SHORT true
#define SCENE2
#define SCENE2_ENABLE_SHORT true
#define SCENE3
#define SCENE3_ENABLE_SHORT true
#define SCENE4
#define SCENE4_ENABLE_SHORT true
BounceSwitch sw1(5, Duration(50), true);
BounceSwitch sw2(6, Duration(50), true);
BounceSwitch sw3(A5, Duration(50), true);
BounceSwitch sw4(A4, Duration(50), true);
#endif

#ifdef TEST
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
//#define RELAY1
#define DIMMER1
BounceSwitch sw1(3, Duration(50), true);
#define NRF24_CE_PIN A5
#define NRF24_CSN_PIN A4
RF24 nrf24Radio(NRF24_CE_PIN, NRF24_CSN_PIN);
PL1167_nRF24 pl1167(nrf24Radio);
//MiLightRelay rel1(pl1167, 0xF2EA, 4);
MiLightDimmer dim1(pl1167, 0xF2EA, 4, false, 10, {.slowDimming=1, .fullBrightness=1});
#endif

#ifdef DIMMER1
DimmerActuator dimmer1(0, dim1, sw1);
#endif
#ifdef DIMMER2
DimmerActuator dimmer2(1, dim2, sw2);
#endif
#ifdef DIMMER3
DimmerActuator dimmer3(2, dim3, sw3);
#endif
#ifdef RELAY1
RelayActuator relay1(0, rel1, sw1);
#endif
#ifdef RELAY2
RelayActuator relay2(1, rel2, sw2);
#endif
#ifdef RELAY3
RelayActuator relay3(2, rel3, sw3);
#endif
#ifdef SCENE1
SceneController scene1(3, sw1, SCENE1_ENABLE_SHORT);
#endif
#ifdef SCENE2
SceneController scene2(4, sw2, SCENE2_ENABLE_SHORT);
#endif
#ifdef SCENE3
SceneController scene3(5, sw3, SCENE3_ENABLE_SHORT);
#endif
#ifdef SCENE4
SceneController scene4(7, sw4, SCENE4_ENABLE_SHORT);
#endif

#ifdef TEMP_PIN
DS18B20RequestableValue tempSensor(TEMP_PIN, 6, Duration(60000));
#endif

/***
 * Dimmable LED initialization method
 */
void setup()
{
  Duration::setClockPrescaler(CLOCK_PRESCALER);
  Serial.begin(115200);
  ActuatorBase::begin();
  #ifdef USE_APDS9930
  myApds.init();
  #endif
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME "-" SKETCH_SUBNAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

  // Register the LED Dimmable Light with the gateway
  ActuatorBase::present();

  wdt_enable(WDTO_8S);
}

/***
 *  Dimmable LED main processing loop 
 */
void loop()
{
  wdt_reset();
  #ifdef USE_APDS9930
  myApds.update();
  #endif
  ActuatorBase::update();
}

void receive(const MyMessage &message) {
  ActuatorBase::receive(message);
}
