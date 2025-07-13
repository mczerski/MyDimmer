// Enable debug prints to serial monitor
//#define MY_DEBUG
#define MY_BAUD_RATE 115200
//#define MYS_TOOLKIT_DEBUG
//#define MYS_TOOLKIT_SERIAL Serial

#define BEDROOM1_SHELF
#define SKETCH_NAME "Dimmer"
#define SKETCH_MAJOR_VER "2"
#define SKETCH_MINOR_VER "12"

// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_RFM69_CS_PIN 4
#define MY_RFM69_NEW_DRIVER
#define MY_RFM69_ATC_MODE_DISABLED
#define MY_RFM69_TX_POWER_DBM 10
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

#ifdef BEDROOM1_SHELF
#define MY_NODE_ID 11
#define SKETCH_SUBNAME "LargeBedroomShelf"
#define USE_APDS9930
#define APDS9930_INT A3
#define APDS9930_NUM 2
#define TEMP_PIN A1
#endif

#ifdef BEDROOM1
#define MY_NODE_ID 35
#undef MY_RFM69_CS_PIN
#define MY_RFM69_CS_PIN 10
#define SKETCH_SUBNAME "LargeBedroom"
#define MY_IS_RFM69HW
#endif

#ifdef BEDROOM1_LAMP
#define MY_NODE_ID 36
#undef MY_RFM69_CS_PIN
#define MY_RFM69_CS_PIN 10
#define SKETCH_SUBNAME "LargeBedroomLamp"
#endif

#ifdef BEDROOM2
#define MY_NODE_ID 34
#undef MY_RFM69_CS_PIN
#define MY_RFM69_CS_PIN 10
#define SKETCH_SUBNAME "SmallBedroom"
#endif

#ifdef LIVINGROOM_SCENE
#define MY_NODE_ID 12
#undef MY_RFM69_CS_PIN
#define MY_RFM69_CS_PIN 10
#undef SKETCH_NAME
#define SKETCH_NAME "Scene"
#define SKETCH_SUBNAME "Livingroom"
#endif

#ifdef BATHROOM1_MOTION
#define MY_NODE_ID 29
#undef MY_RFM69_CS_PIN
#define MY_RFM69_CS_PIN 10
#define MY_OTA_FLASH_JDECID 0xC840
#undef SKETCH_NAME
#define SKETCH_NAME "Motion"
#define SKETCH_SUBNAME "LargeBathroom"
#endif

#ifdef BATHROOM2_MOTION
#define MY_NODE_ID 30
#undef MY_RFM69_CS_PIN
#define MY_RFM69_CS_PIN 10
#define MY_OTA_FLASH_JDECID 0xC840
#undef SKETCH_NAME
#define SKETCH_NAME "Motion"
#define SKETCH_SUBNAME "SmallBathroom"
#endif

#ifdef TEST
HardwareSerial mySerial(PA3, PA2);
#define MY_SERIALDEVICE mySerial
#undef MY_RADIO_RFM69
#undef MY_OTA_FIRMWARE_FEATURE
#define MY_RS485
#define MY_RS485_DE_PIN PA8
#define MY_RS485_BAUD_RATE 115200
#define MY_RS485_HWSERIAL Serial
#define MY_DEBUG_VERBOSE_RS485

#define MY_NODE_ID 13
#define MY_DEBUG
#define MY_DEBUG_VERBOSE_RFM69
#define MY_DEBUG_VERBOSE_TRANSPORT
#define MY_SIGNAL_REPORT_ENABLED
#undef MY_RFM69_CS_PIN
#define MY_RFM69_CS_PIN 10
#define MY_RFM69_IRQ_PIN 2
#undef SKETCH_NAME
#define SKETCH_NAME "Test"
#define SKETCH_SUBNAME ""
#endif

#if defined(ARDUINO_ARCH_AVR)
#include <avr/wdt.h>
#endif
#include <MySensors.h>
#include <MySensorsToolkit.h>
#include "BounceSwitch.h"
#include "AnalogBounceSwitch.h"
#include "APDS9930Switch.h"
#include "DS18B20RequestableValue.h"
#include "MiLightRelay.h"
#include "MiLightDimmer.h"
#include "PWMSwitch.h"
#include <MySensorsToolkit/Actuator/DimmerActuator.h>
#include <MySensorsToolkit/Actuator/RelayActuator.h>
#include <MySensorsToolkit/Actuator/SceneController.h>
#include <MySensorsToolkit/Actuator/MotionSensor2.h>

using namespace mys_toolkit;

#ifdef USE_APDS9930
MyAPDS9930 myApds(APDS9930_INT, APDS9930_NUM);
#endif

#ifdef KITCHEN
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
#define DIMMER1
#define DIMMER2
#define DIMMER3
BounceSwitch sw1(A1, Duration(50), true);
BounceSwitch sw2(A2, Duration(50), true);
BounceSwitch sw3(A3, Duration(50), true);
CwWwDimmerDriver dimDrv1(9, 10, false);
PushPullDimmer dim1(dimDrv1, 10, {.slowDimming=1, .fullBrightness=1});
SimpleDimmerDriver dimDrv2(3, false);
PushPullDimmer dim2(dimDrv2, 10, {.slowDimming=1, .fullBrightness=1});
#define NRF24_CE_PIN 7
#define NRF24_CSN_PIN 6
RF24 nrf24Radio(NRF24_CE_PIN, NRF24_CSN_PIN);
PL1167_nRF24 pl1167(nrf24Radio);
MiLightDimmerDriver dimDrv3(pl1167, 0xF2EA, 4, false);
PushPullDimmer dim3(dimDrv3, 10, {.slowDimming=1, .fullBrightness=1});
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

#ifdef BEDROOM1_SHELF
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
#define DIMMER1
#define DIMMER2
#define SCENE1
#define SCENE1_ENABLE_SHORT false
#define SCENE2
#define SCENE2_ENABLE_SHORT false
APDS9930Switch sw1(myApds, 0);
APDS9930Switch sw2(myApds, 1);
SimpleDimmerDriver dimDrv1(3, true);
SimpleDimmerDriver dimDrv2(5, true);
PushPullDimmer dim1(dimDrv1, 10, {.slowDimming=0, .fullBrightness=0});
PushPullDimmer dim2(dimDrv2, 10, {.slowDimming=0, .fullBrightness=0});
#endif

#ifdef BEDROOM1
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
#define DIMMER1
#define SCENE2
#define SCENE2_ENABLE_SHORT true
BounceSwitch sw1(4, Duration(50), true);
BounceSwitch sw2(3, Duration(50), true);
SimpleDimmer dim1(5, true, 10, {.slowDimming=1, .fullBrightness=1});
#endif

#ifdef BEDROOM1_LAMP
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
#define DIMMER1
PWMSwitch sw1(3, false);
SimpleDimmerDriver dimDrv1(5, false);
PushDimmer dim1(dimDrv1, 10);
#endif

#ifdef BEDROOM2
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
#define DIMMER1
#define SCENE2
#define SCENE2_ENABLE_SHORT true
BounceSwitch sw1(4, Duration(50), true);
BounceSwitch sw2(3, Duration(50), true);
SimpleDimmer dim1(5, true, 10, {.slowDimming=1, .fullBrightness=1});
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

#ifdef BATHROOM1_MOTION
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
MotionSensor2 motion1(0, 3);
#endif

#ifdef BATHROOM2_MOTION
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
MotionSensor2 motion1(0, 3);
#endif

#ifdef TEST
#define CLOCK_PRESCALER CLOCK_PRESCALER_1
#define DIMMER1
BounceSwitch sw1(PB9, Duration(50), true);
SimpleDimmer dim1(PC13, true, 10, {.slowDimming = 1, .fullBrightness = 1});
//#define SCENE1
//#define SCENE1_ENABLE_SHORT true
//BounceSwitch sw1(PA0, Duration(50), true);
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
  Serial.println("Hello");
  Duration::setClockPrescaler(CLOCK_PRESCALER);
  ActuatorBase::begin();
#ifdef USE_APDS9930
  myApds.init();
#endif
#if defined(ARDUINO_ARCH_AVR)
  wdt_enable(WDTO_8S);
#endif
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME "-" SKETCH_SUBNAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

  // Register the LED Dimmable Light with the gateway
  ActuatorBase::present();

}

/***
 *  Dimmable LED main processing loop 
 */
void loop()
{
#if defined(ARDUINO_ARCH_AVR)
  wdt_reset();
#endif
#ifdef USE_APDS9930
  myApds.update();
#endif
  ActuatorBase::update();
}

void receive(const MyMessage &message) {
  ActuatorBase::receive(message);
}
