// Enable debug prints to serial monitor
//#define MY_DEBUG
//#define MY_MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_RF24_CE_PIN 7
#define MY_RF24_CS_PIN 4
#define MY_RF24_PA_LEVEL RF24_PA_MAX
#define MY_RF24_CHANNEL 100
//#define MY_RADIO_RFM69
#define MY_REPEATER_FEATURE
#define MY_OTA_FIRMWARE_FEATURE
#define MY_TRANSPORT_WAIT_READY_MS 1

#define MY_NODE_ID 6

#include "MyMySensors/MyMySensors.h"
#include <Bounce2.h>
#include <SoftTimer.h>
#include <DallasTemperature.h>
#include <OneWire.h>

using namespace mymysensors;

#define SKETCH_NAME "Dimmer"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "6"

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

class Dimmer {
  enum State {
    OFF,
    ON,
    DIMMING_UP,
    SLOW_DIMMING_UP,
    DIMMING_DOWN,
    SLOW_DIMMING_DOWN
  };
  uint8_t pin_;
  bool lastPinValue_;
  State state_;
  uint8_t currentLevel_;
  uint8_t requestedLevel_;
  unsigned long nextChangeTime_;
  bool inverted_;
  unsigned long lastPinRiseTime_;

  int fadeDelay_()
  {
    if (currentLevel_ < 10)
      return 25;
    else if (currentLevel_ < 20)
      return 25;
    else if (currentLevel_ < 50)
      return 10;
    else if (currentLevel_ < 100)
      return 5;
    else
      return 3;
  }

  void setLevel_() {
    analogWrite(pin_, inverted_ ? 255 - currentLevel_ : currentLevel_);
  }

  bool updateLevel_() {
    if (isInIdleState_())
      return false;
    unsigned long currentTime = millis();
    if (currentTime < nextChangeTime_)
      return false;
    handleDimming_();
    setLevel_();
    int fadeDelay = fadeDelay_();
    if (isInSlowDimming_())
      fadeDelay *= 3;
    nextChangeTime_ = nextChangeTime_ + fadeDelay;
    return isInIdleState_();
  }

  void handleDimming_() {
    if (state_ == DIMMING_DOWN) {
      if (currentLevel_ > requestedLevel_) {
        currentLevel_--;
      }
      if (currentLevel_ <= requestedLevel_) {
        currentLevel_ = requestedLevel_;
        state_ = OFF;
      }
    }
    else if (state_ == DIMMING_UP) {
      if (currentLevel_ < requestedLevel_) {
        currentLevel_++;
      }
      if (currentLevel_ >= requestedLevel_) {
        currentLevel_ = requestedLevel_;
        state_ = ON;
      }
    }
    else if (state_ == SLOW_DIMMING_UP) {
      if (currentLevel_ == 255) {
        currentLevel_--;
        state_ = SLOW_DIMMING_DOWN;
      }
      else {
        currentLevel_++;
      }
    }
    else if (state_ == SLOW_DIMMING_DOWN) {
      if (currentLevel_ == 1) {
        currentLevel_++;
        state_ = SLOW_DIMMING_UP;
      }
      else {
        currentLevel_--;
      }
    }
  }

  bool isRising_(bool pinValue) {
    return pinValue == true and lastPinValue_ == false;
  }

  bool isHeldLongEnough_(bool pinValue) {
    return pinValue == true and lastPinValue_ == true and millis() > lastPinRiseTime_ + 500;
  }

  bool isFalling(bool pinValue) {
    return pinValue == false and lastPinValue_ == true;
  }

  bool isInSlowDimming_() {
    return state_ == SLOW_DIMMING_DOWN or state_ == SLOW_DIMMING_UP;
  }

  bool isInIdleState_() {
    return state_ == OFF or state_ == ON;
  }

  void triggerLevelChange_() {
    nextChangeTime_ = millis();
  }

  void startSlowDimming_() {
    triggerLevelChange_();
    if (currentLevel_ > 255/2) {
      state_ = SLOW_DIMMING_DOWN;
    }
    else {
      state_ = SLOW_DIMMING_UP;
    }
  }

  void startDimming_() {
    triggerLevelChange_();
    if (state_ == OFF or state_ == DIMMING_DOWN) {
      requestedLevel_ = 255;
      state_ = DIMMING_UP;
    }
    else if (state_ == ON or state_ == DIMMING_UP) {
      requestedLevel_ = 0;
      state_ = DIMMING_DOWN;
    }
  }

  void requestDimming_(uint8_t level) {
    triggerLevelChange_();
    if (level < currentLevel_) {
      state_ = DIMMING_DOWN;
    }
    else {
      state_ = DIMMING_UP;
    }
    requestedLevel_ = level;
  }

  void stopSlowDimming_() {
      state_ = ON;
      requestedLevel_ = currentLevel_;
  }

public:
  Dimmer(uint8_t pin, bool inverted)
    : pin_(pin), lastPinValue_(false),
      state_(OFF), currentLevel_(0),
      requestedLevel_(0), nextChangeTime_(0),
      inverted_(inverted) {
    setLevel_();
  }

  bool update(bool value) {
    if (isRising_(value)) {
      lastPinRiseTime_ = millis();
    }
    else if (isHeldLongEnough_(value)) {
      if (not isInSlowDimming_()) {
        triggerLevelChange_();
        startSlowDimming_();
      }
    }
    else if (isFalling(value)) {
      if (isInSlowDimming_()) {
        stopSlowDimming_();
        lastPinValue_ = value;
        return true;
      }
      else {
        startDimming_();
      }
    }
    lastPinValue_ = value;
    return updateLevel_();
  }

  void request(uint8_t value) {
    if (isInSlowDimming_()) {
      return;
    }
    requestDimming_(value);
  }

  uint8_t getLevel() {
    return currentLevel_;
  }

};

class Switch {
  Bounce switch_;
  uint8_t activeLow_;
public:
  Switch(uint8_t pin, unsigned long interval_ms, bool activeLow = false) : switch_(pin, interval_ms), activeLow_(activeLow) {
    pinMode(pin, INPUT);
    digitalWrite(pin, activeLow_ ? HIGH : LOW);
    switch_.update();
  }
  bool update() {
    switch_.update();
    return activeLow_ ? !switch_.read() : switch_.read();
  }
};

class MyDimmerSwitch {
  Dimmer dim_;
  Switch sw_;
  uint8_t childId_;
  MyMessage dimmerMsg_;
  MyMessage lightMsg_;
  static const uint8_t MAX_DIMMERS = 10;
  static uint8_t dimmersCount_;
  static MyDimmerSwitch *dimmers_[MAX_DIMMERS];

  static uint8_t fromPercentage_(uint8_t percentage) {
    return uint8_t(round(255.0*percentage/100));
  }
  static uint8_t fromLevel_(uint8_t level) {
    return uint8_t(round(100.0*level/255));
  }

public:
  MyDimmerSwitch(uint8_t dimPin, bool inverted, uint8_t switchPin, unsigned long interval_ms, bool activeLow)
    : dim_(dimPin, inverted),
      sw_(switchPin, interval_ms, activeLow),
      childId_(dimmersCount_),
      dimmerMsg_(childId_, V_DIMMER),
      lightMsg_(childId_, V_LIGHT) {
    if (dimmersCount_ < MAX_DIMMERS)
      dimmers_[dimmersCount_++] = this;
  }
  static void request() {
    for (size_t i=0; i<dimmersCount_; i++)
      ::request(dimmers_[i]->childId_, V_DIMMER);
  }
  static void present() {
    for (size_t i=0; i<dimmersCount_; i++)
      ::present(dimmers_[i]->childId_, S_DIMMER);
  }
  static void update() {
    for (size_t i=0; i<dimmersCount_; i++) {
      MyDimmerSwitch * dimmer = dimmers_[i];
      if (dimmer->dim_.update(dimmer->sw_.update()))
        dimmer->sendCurrentLevel();
    }
  }
  void sendCurrentLevel() {
    uint8_t percentage = fromLevel_(dim_.getLevel());
    sendValue(lightMsg_, percentage > 0 ? 1 : 0);
    sendValue(dimmerMsg_, percentage);
  }
  void requestLevel(uint8_t level) {
    dim_.request(level);
  }
  uint8_t getLevel() {
    return dim_.getLevel();
  }
  static bool receive(const MyMessage &message) {
    if (message.sensor > dimmersCount_-1)
      return false;
    if (message.isAck())
      return true;
    if (message.type == V_LIGHT || message.type == V_DIMMER) {
      MyDimmerSwitch * dimmer = dimmers_[message.sensor];
      if (mGetCommand(message) == C_REQ) {
        dimmer->sendCurrentLevel();
      }
      else if (mGetCommand(message) == C_SET) {
        //  Retrieve the power or dim level from the incoming request message
        int requestedPercentage = atoi( message.data );
    
        // Adjust incoming level if this is a V_LIGHT variable update [0 == off, 1 == on]
        requestedPercentage *= ( message.type == V_LIGHT ? 100 : 1 );
    
        // Clip incoming level to valid range of 0 to 100
        requestedPercentage = requestedPercentage > 100 ? 100 : requestedPercentage;
        requestedPercentage = requestedPercentage < 0   ? 0   : requestedPercentage;

        #ifdef MY_MY_DEBUG
        Serial.print("Changing dimmer [");
        Serial.print(message.sensor);
        Serial.print("] level to ");
        Serial.print(requestedPercentage);
        Serial.print( ", from " );
        Serial.println(MyDimmerSwitch::fromLevel_(dimmer->dim_.getLevel()));
        #endif
  
        dimmer->requestLevel(MyDimmerSwitch::fromPercentage_(requestedPercentage));
      }
    }
    return true;
  }
};

uint8_t MyDimmerSwitch::dimmersCount_ = 0;
MyDimmerSwitch * MyDimmerSwitch::dimmers_[MAX_DIMMERS];

MyDimmerSwitch dimmer1(3, true, A1, 50, true);
MyDimmerSwitch dimmer2(5, true, A2, 50, true);
MyDimmerSwitch dimmer3(6, true, A3, 50, true);
MyDimmerSwitch dimmer4(9, false, A1, 50, true);
MyDimmerSwitch dimmer5(10, false, A2, 50, true);

template <typename ValueType, ValueType (*ReadValueCb)(), int16_t (*StartMeasurementCb)()>
class MyRequestingValue : public EventBase
{
  MyMessage msg_;
  uint8_t childId_;
  uint8_t sensorType_;
  ValueType value_;
  SoftTimer timer_;
  int32_t interval_;
  void scheduleEvent(boolean (*cb)(EventBase*), int32_t delayMs)
  {
    this->period = 0;
    this->repeatCount = 1;
    this->nextTriggerTime = millis() + delayMs;
    this->callback = cb;
    timer_.addEvent(this);
  }
  static boolean readValue_(EventBase* event)
  {
    MyRequestingValue* myRequestingValue = static_cast<MyRequestingValue*>(event);
    myRequestingValue->value_ = (*ReadValueCb)();
    #ifdef MY_MY_DEBUG
    Serial.print("readValue: ");
    Serial.print(myRequestingValue->value_);
    Serial.print(" next measurement: ");
    Serial.print(myRequestingValue->interval_);
    #endif
    myRequestingValue->scheduleEvent(MyRequestingValue::startMeasurement_, myRequestingValue->interval_);
    return true;
  }
  static boolean startMeasurement_(EventBase* event)
  {
    MyRequestingValue* myRequestingValue = static_cast<MyRequestingValue*>(event);
    int16_t conversionTime = (*StartMeasurementCb)();
    #ifdef MY_MY_DEBUG
    Serial.print("startMeasurement conversionTime: ");
    Serial.println(conversionTime);
    #endif
    myRequestingValue->scheduleEvent(MyRequestingValue::readValue_, conversionTime);
    return true;
  }
public:
  MyRequestingValue(uint8_t sensor, uint8_t type, uint8_t sensorType, int32_t interval)
    : msg_(sensor, type), childId_(sensor), sensorType_(sensorType), interval_(interval)
  {}
  void presentValue() {
    present(childId_, sensorType_);
  }
  void begin() {
    scheduleEvent(startMeasurement_, 0);
  }
  void update() {
    timer_.update();
  }
  bool receive(const MyMessage &message) {
    if (message.sensor != childId_)
      return false;
    if (message.isAck())
      return true;
    if (message.type == msg_.type and mGetCommand(message) == C_REQ) {
      sendValue(msg_, value_);
    }
    return true;
  }
};

OneWire oneWire(18);
DallasTemperature tempSensor(&oneWire);

float readTempMeasurement()
{
  return tempSensor.getTempCByIndex(0);
}

int16_t startTempMeasurement()
{
  tempSensor.requestTemperatures();
  return tempSensor.millisToWaitForConversion(tempSensor.getResolution());
}

MyRequestingValue<float, readTempMeasurement, startTempMeasurement> temperature(6, V_TEMP, S_TEMP, 60000);

/***
 * Dimmable LED initialization method
 */
void setup()
{
  Serial.begin(115200);
  setPwmFrequency(3, 64);
  setPwmFrequency(6, 64);
  setPwmFrequency(10, 64);
  tempSensor.begin();
  tempSensor.setWaitForConversion(false);
  temperature.begin();
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

  // Register the LED Dimmable Light with the gateway
  MyDimmerSwitch::present();
  temperature.presentValue();
}

/***
 *  Dimmable LED main processing loop 
 */
void loop()
{
  MyDimmerSwitch::update();
  temperature.update();
}

void receive(const MyMessage &message) {
  if (message.isAck())
    return;
  bool handeled = MyDimmerSwitch::receive(message);
  if (not handeled) {
    temperature.receive(message);
  }
}

