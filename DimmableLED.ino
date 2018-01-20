#include "prescaler.h"
#include <limits.h>

// Enable debug prints to serial monitor
//#define MY_DEBUG
//#define MY_MY_DEBUG

#define BEDROOM
#define SKETCH_NAME "Dimmer"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "24"

#ifdef KITCHEN
#define MY_NODE_ID 8
#define TEMP_PIN A4
#define APDS9930_NUM 1
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

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_RF24_CE_PIN 7
#define MY_RF24_CS_PIN 4
#define MY_RF24_PA_LEVEL RF24_PA_MAX
#define MY_RF24_CHANNEL 100
//#define MY_RADIO_RFM69
#define MY_OTA_FIRMWARE_FEATURE
#define MY_TRANSPORT_WAIT_READY_MS 1

#include "MyMySensors/MyMySensors.h"
#include <Bounce2.h>
#include <SoftTimer.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <APDS9930.h>
#include <AnalogMultiButton.h>

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

class MyDuration
{
public:
  typedef unsigned long duration_ms_t;
  explicit MyDuration(duration_ms_t duration) : duration_(rescaleDuration(duration)) {}
  explicit MyDuration() : duration_(millis()) {}
  duration_ms_t getMilis() const {
    return rescaleTime(duration_);
  }
  bool operator<(const MyDuration &other){
    duration_ms_t difference = duration_ - other.duration_;
    return difference >= ULONG_MAX/2;
  }
  void operator+=(const MyDuration &other) {
    duration_ += other.duration_;
  }
  void operator*=(int factor) {
    duration_ *= factor;
  }
  MyDuration operator+(const MyDuration &other) {
    MyDuration result = *this;
    result += other;
    return result;
  }
private:
  duration_ms_t duration_;
};

class MyMySensorsBase
{
  virtual void begin_() {}
  virtual void update_() {}
  virtual void firstUpdate_() {}
  virtual void receive_(const MyMessage &) {}
  uint8_t sensorId_;
  uint8_t sensorType_;
  static constexpr uint8_t MAX_SENSORS = 10;
  static uint8_t sensorsCount_;
  static MyMySensorsBase* sensors_[MAX_SENSORS];
  static bool loopCalled_;
  static SoftTimer timer_;

  static boolean heartbeatCb(EventBase*) {
    sendHeartbeat();
    return false;
  }

public:
  MyMySensorsBase(uint8_t sensorId, uint8_t sensorType)
    : sensorId_(sensorId), sensorType_(sensorType)
  {
    if (sensorsCount_ < MAX_SENSORS)
      sensors_[sensorsCount_++] = this;
  }
  static void begin() {
    timer_.every(rescaleDuration(60000), &MyMySensorsBase::heartbeatCb, rescaleDuration(60000));
    for (size_t i=0; i<sensorsCount_; i++)
      sensors_[i]->begin_();
  }
  static void present() {
    for (size_t i=0; i<sensorsCount_; i++)
      ::present(sensors_[i]->sensorId_, sensors_[i]->sensorType_);
  }
  static void update() {
    timer_.update();
    for (size_t i=0; i<sensorsCount_; i++)
      sensors_[i]->update_();
    if (not loopCalled_) {
      checkTransport();
      for (size_t i=0; i<sensorsCount_; i++)
        sensors_[i]->firstUpdate_();
      loopCalled_ = true;
    }
  }
  static void receive(const MyMessage &message) {
    if (message.isAck()) {
      MyMyMessage::setSent(message);
      return;
    }
    for (size_t i=0; i<sensorsCount_; i++)
      if (sensors_[i]->sensorId_ == message.sensor)
        sensors_[i]->receive_(message);
  }
  static int16_t addEvent(EventBase* evt) {
    return timer_.addEvent(evt);
  }
};

uint8_t MyMySensorsBase::sensorsCount_ = 0;
MyMySensorsBase * MyMySensorsBase::sensors_[];
bool MyMySensorsBase::loopCalled_ = false;
SoftTimer MyMySensorsBase::timer_;

struct Functions {
  uint8_t slowDimming : 1;
  uint8_t fullBrightness : 1;
};

class Dimmer {
  enum State {
    OFF,
    ON,
    DIMMING_UP,
    SLOW_DIMMING_UP,
    DIMMING_DOWN,
    SLOW_DIMMING_DOWN
  };
  bool lastPinValue_;
  State state_;
  uint8_t currentLevel_;
  uint8_t requestedLevel_;
  uint8_t lastLevel_;
  MyDuration nextChangeTime_;
  bool inverted_;
  MyDuration lastPinRiseTime_;
  uint8_t dimmSpeed_;
  static constexpr uint8_t maxDimmSpeed_ = 20;
  Functions functions_;

  MyDuration fadeDelay_()
  {
    auto delayFactor = maxDimmSpeed_ - dimmSpeed_ + 1;
    if (currentLevel_ < 1*delayFactor)
      return MyDuration(25);
    else if (currentLevel_ < 2*delayFactor)
      return MyDuration(25);
    else if (currentLevel_ < 5*delayFactor)
      return MyDuration(10);
    else if (currentLevel_ < 10*delayFactor)
      return MyDuration(5);
    else
      return MyDuration(3);
  }

  virtual void setLevel_(uint8_t level) = 0;

  bool updateLevel_() {
    if (isInIdleState_())
      return false;
    MyDuration currentTime;
    if (currentTime < nextChangeTime_)
      return false;
    handleDimming_();
    setLevel_(currentLevel_);
    MyDuration fadeDelay = fadeDelay_();
    if (isInSlowDimming_())
      fadeDelay *= 3;
    nextChangeTime_ += fadeDelay;
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
        lastLevel_ = requestedLevel_;
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
    return pinValue == true and lastPinValue_ == true and lastPinRiseTime_ + MyDuration(2000) < MyDuration();
  }

  bool isFalling(bool pinValue) {
    return pinValue == false and lastPinValue_ == true;
  }

  bool isLongPress() {
    return lastPinRiseTime_ + MyDuration(500) < MyDuration();
  }

  bool isInSlowDimming_() {
    return state_ == SLOW_DIMMING_DOWN or state_ == SLOW_DIMMING_UP;
  }

  bool isInIdleState_() {
    return state_ == OFF or state_ == ON;
  }

  void triggerLevelChange_() {
    nextChangeTime_ = MyDuration();
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

  void stopSlowDimming_() {
      requestedLevel_ = currentLevel_;
      lastLevel_ = currentLevel_;
      state_ = ON;
  }

protected:
  void driveLedPin_(uint8_t pin, uint8_t level) {
    analogWrite(pin, inverted_ ? 255 - level : level);
  }

  void init_() {
    setLevel_(currentLevel_);
  }

public:
  Dimmer(bool inverted, uint8_t dimmSpeed, Functions functions)
    : lastPinValue_(false), state_(OFF), currentLevel_(0),
      requestedLevel_(0), lastLevel_(0), inverted_(inverted),
      dimmSpeed_(min(maxDimmSpeed_, dimmSpeed)),
      functions_(functions) {}

  virtual ~Dimmer() {}

  bool update(bool value) {
    if (isRising_(value)) {
      if (not functions_.slowDimming and not functions_.fullBrightness)
        set(state_ == OFF);
      else
        lastPinRiseTime_ = MyDuration();
    }
    else if (isHeldLongEnough_(value)) {
      if (not isInSlowDimming_() and functions_.slowDimming) {
        triggerLevelChange_();
        startSlowDimming_();
      }
    }
    else if (isFalling(value)) {
      if (isInSlowDimming_() and functions_.slowDimming) {
        stopSlowDimming_();
        lastPinValue_ = value;
        return true;
      }
      else if (isLongPress() and functions_.fullBrightness) {
        request(255);
      }
      else if (functions_.slowDimming or functions_.fullBrightness) {
        set(state_ == OFF or state_ == DIMMING_DOWN);
      }
    }
    lastPinValue_ = value;
    return updateLevel_();
  }

  void request(uint8_t value) {
    if (isInSlowDimming_() or value == currentLevel_) {
      return;
    }
    triggerLevelChange_();
    if (value < currentLevel_) {
      state_ = DIMMING_DOWN;
    }
    else {
      state_ = DIMMING_UP;
    }
    requestedLevel_ = value;
  }

  void set(bool on) {
    if (on) {
      request(lastLevel_ ? lastLevel_ : 255);
    }
    else {
      request(0);
    }
  }

  uint8_t getLevel() {
    return currentLevel_;
  }

};

class CwWwDimmer: public Dimmer {
  uint8_t wwPin_;
  uint8_t cwPin_;

  void setLevel_(uint8_t level) override {
    uint8_t wwLevel = level < 128 ? 2*level : 255;
    uint8_t cwLevel = level > 128 ? 2*(level - 128) : 0;
    driveLedPin_(wwPin_, wwLevel);
    driveLedPin_(cwPin_, cwLevel);
  }

public:
  CwWwDimmer(uint8_t wwPin, uint8_t cwPin, bool inverted, uint8_t dimmSpeed, Functions functions)
    : Dimmer(inverted, dimmSpeed, functions), wwPin_(wwPin), cwPin_(cwPin) {
      init_();
  }
};

class SimpleDimmer: public Dimmer {
  uint8_t pin_;

  void setLevel_(uint8_t level) override {
    driveLedPin_(pin_, level);
  }

public:
  SimpleDimmer(uint8_t pin, bool inverted, uint8_t dimmSpeed, Functions functions)
    : Dimmer(inverted, dimmSpeed, functions), pin_(pin) {
      init_();
  }
};

class Switch {
  uint8_t activeLow_;
  virtual bool doUpdate_() = 0;
public:
  Switch(bool activeLow = false) : activeLow_(activeLow) {}
  bool update() {
    bool state = doUpdate_();
    return activeLow_ ? !state : state;
  }
};

class BounceSwitch : public Switch {
  Bounce switch_;
  bool doUpdate_() override {
    switch_.update();
    return switch_.read();
  }
public:
  BounceSwitch(uint8_t pin, MyDuration interval_ms, bool activeLow = false) : Switch(activeLow) {
    switch_.attach(pin, activeLow ? INPUT_PULLUP : INPUT);
    switch_.interval(interval_ms.getMilis());
  }
};

class AnalogBounceSwitch : public Switch {
  AnalogMultiButton button_;
  const int buttonValues_[1] = {0};
  bool doUpdate_() override {
    button_.update();
    return !button_.isPressed(0);
  }
public:
  AnalogBounceSwitch(uint8_t pin, MyDuration interval_ms, bool activeLow = false) : Switch(activeLow), button_(pin, 1, buttonValues_, interval_ms.getMilis()) {
    button_.update();
  }
};

class MyAPDS9930 {
  uint8_t intPin_;
  APDS9930 apds_[APDS9930_NUM];
  uint8_t apdsInts_;
  static const int PCAADDR = 0x70;
  static const int PROX_INT_HIGH = 900; // Proximity level for interrupt
  static const int PROX_INT_LOW = 0;  // No far interrupt

  void pcaSelect_(uint8_t i) {
    #if APDS9930_NUM == 1
    (void)i;
    return;
    #else
    if (i > 3) return;
    Wire.beginTransmission(PCAADDR);
    Wire.write(4 + i);
    Wire.endTransmission();
    #endif
  }
  uint8_t pcaGet_() {
    #if APDS9930_NUM == 1
    return digitalRead(intPin_) == LOW ? 1 : 0;
    #else
    Wire.requestFrom(PCAADDR, 1);
    return Wire.read() >> 4;
    #endif
  }
  void init_(uint8_t i) {
    pcaSelect_(i);
    bool status = apds_[i].init();
    #ifdef MY_MY_DEBUG
    Serial.print(F("APDS-9930 initializing sensor #"));
    Serial.println(i);
    if (status) {
      Serial.println(F("APDS-9930 initialization complete"));
    } else {
      Serial.println(F("Something went wrong during APDS-9930 init!"));
    }
    #endif
    status = apds_[i].enableProximitySensor(true);
    #ifdef MY_MY_DEBUG
    if (status) {
      Serial.println(F("Proximity sensor is now running"));
    } else {
      Serial.println(F("Something went wrong during sensor init!"));
    }
    #endif
    status = apds_[i].setProximityGain(PGAIN_1X);
    #ifdef MY_MY_DEBUG
    if (!status) {
      Serial.println(F("Something went wrong trying to set PGAIN"));
    }
    #endif
    status = apds_[i].setProximityIntLowThreshold(PROX_INT_LOW);
    #ifdef MY_MY_DEBUG
    if (!status) {
      Serial.println(F("Error writing low threshold"));
    }
    #endif
    status = apds_[i].setProximityIntHighThreshold(PROX_INT_HIGH);
    #ifdef MY_MY_DEBUG
    if (!status) {
      Serial.println(F("Error writing high threshold"));
    }
    #endif
    (void)status;
  }
  void update_(uint8_t i) {
    pcaSelect_(i);
    uint16_t proximity_data = 0;
    #ifdef MY_MY_DEBUG
    Serial.print("Reading sensor #");
    Serial.println(i);
    #endif
    if (!apds_[i].readProximity(proximity_data)) {
      #ifdef MY_MY_DEBUG
      Serial.println("Error reading proximity value");
      #endif
    } else {
      #ifdef MY_MY_DEBUG
      Serial.print("Proximity detected! Level: ");
      Serial.println(proximity_data);
      #endif
    }
    if (proximity_data < PROX_INT_HIGH) {
      if (!apds_[i].clearProximityInt()) {
        #ifdef MY_MY_DEBUG
        Serial.println("Error clearing interrupt");
        #endif
      }
    }
  }

public:
  MyAPDS9930(uint8_t intPin) : intPin_(intPin), apdsInts_(0) {}
  void init() {
    pinMode(intPin_, INPUT_PULLUP);
    for (uint8_t i=0; i<APDS9930_NUM; i++)
      init_(i);
  }
  void update() {
    apdsInts_ = pcaGet_();
    if (apdsInts_) {
      for (uint8_t i=0; i<APDS9930_NUM; i++) {
        if (apdsInts_ & (1 << i)) {
          update_(i);
        }
      }
    }
  }
  bool getInt(uint8_t i) const {
    return apdsInts_ & (1 << i);
  }
};

class APDS9930Switch : public Switch {
  const MyAPDS9930 &myApds_;
  uint8_t apdsNo_;
  bool doUpdate_() override {
    return myApds_.getInt(apdsNo_);
  }
public:
  APDS9930Switch(const MyAPDS9930 &myApds, uint8_t apdsNo) : Switch(false), myApds_(myApds), apdsNo_(apdsNo) {}
};

class MyDimmerSwitch : public MyMySensorsBase
{
  Dimmer &dim_;
  Switch &sw_;
  MyMyMessage dimmerMsg_;
  MyMyMessage lightMsg_;
  static uint8_t fromPercentage_(uint8_t percentage) {
    return uint8_t(round(255.0*percentage/100));
  }
  static uint8_t fromLevel_(uint8_t level) {
    return uint8_t(round(100.0*level/255));
  }
  void sendCurrentLevel_() {
    uint8_t percentage = fromLevel_(dim_.getLevel());
    lightMsg_.send(percentage > 0 ? 1 : 0);
    dimmerMsg_.send(percentage);
    #ifdef MY_MY_DEBUG
    Serial.print("sendCurrentLevel ");
    Serial.print(percentage);
    Serial.print(" for child id ");
    Serial.println(lightMsg_.sensor);
    #endif
  }
  void firstUpdate_() override {
    sendCurrentLevel_();
  }
  void update_() override {
    if (dim_.update(sw_.update()))
      sendCurrentLevel_();
  }
  void receive_(const MyMessage &message) override {
    if (mGetCommand(message) == C_REQ) {
      sendCurrentLevel_();
    }
    else if (mGetCommand(message) == C_SET) {
      //  Retrieve the power or dim level from the incoming request message
      int requestedValue = atoi(message.data);

      if (message.type == V_DIMMER) {    
        // Clip incoming level to valid range of 0 to 100
        requestedValue = requestedValue > 100 ? 100 : requestedValue;
        requestedValue = requestedValue < 0   ? 0   : requestedValue;

        #ifdef MY_MY_DEBUG
        Serial.print("Changing dimmer [");
        Serial.print(message.sensor);
        Serial.print("] level to ");
        Serial.print(requestedValue);
        Serial.print( ", from " );
        Serial.println(fromLevel_(dim_.getLevel()));
        #endif

        dim_.request(fromPercentage_(requestedValue));
      }
      else if (message.type == V_STATUS) {
        dim_.set(requestedValue);
      }
    }
  }
public:
  MyDimmerSwitch(uint8_t sensorId, Dimmer &dim, Switch &sw)
    : MyMySensorsBase(sensorId, S_DIMMER), 
      dim_(dim),
      sw_(sw),
      dimmerMsg_(sensorId, V_DIMMER),
      lightMsg_(sensorId, V_STATUS)
  {}
};

class MyRelaySwitch : public MyMySensorsBase
{
  bool state_;
  bool prevSwState_;
  int relayPin_;
  Switch &sw_;
  MyMyMessage lightMsg_;
  void sendCurrentState_() {
    lightMsg_.send(state_);
    #ifdef MY_MY_DEBUG
    Serial.print("sendCurrentState ");
    Serial.print(state_);
    Serial.print(" for child id ");
    Serial.println(lightMsg_.sensor);
    #endif
  }
  void firstUpdate_() override {
    sendCurrentState_();
  }
  void update_() override {
    bool currSwState = sw_.update();
    if (prevSwState_ != currSwState && state_ != currSwState) {
        state_ = currSwState;
        sendCurrentState_();
    }
    prevSwState_ = currSwState;
    digitalWrite(relayPin_, !state_);
  }
  void receive_(const MyMessage &message) override {
    if (mGetCommand(message) == C_REQ) {
      sendCurrentState_();
    }
    else if (mGetCommand(message) == C_SET) {
      if (message.type == V_STATUS) {    
        bool requestedState = message.getBool();

        #ifdef MY_MY_DEBUG
        Serial.print("Changing relay [");
        Serial.print(message.sensor);
        Serial.print("] state to ");
        Serial.print(requestedState);
        Serial.print( ", from " );
        Serial.println(state_);
        #endif

        state_ = requestedState;
        sendCurrentState_();
      }
    }
  }
public:
  MyRelaySwitch(uint8_t sensorId, Switch &sw, int relayPin)
    : MyMySensorsBase(sensorId, S_BINARY),
      state_(false),
      prevSwState_(false),
      relayPin_(relayPin),
      sw_(sw),
      lightMsg_(sensorId, V_STATUS)
  {
    pinMode(relayPin_, OUTPUT);
  }
};

class MySceneController : public MyMySensorsBase
{
  enum State {
    WAITING_FOR_RISING,
    WAITING_FOR_SCENE,
    WAITING_FOR_FALLING
  };
  State state_;
  bool prevSwState_;
  MyDuration lastSwRiseTime_;
  Switch &sw_;
  MyMyMessage sceneMsg_;
  bool enableShortPress_;
  bool isRising_(bool swState) {
    return swState == true and prevSwState_ == false;
  }
  bool isHeldLongEnough_(bool swState) {
    return swState == true and prevSwState_ == true and lastSwRiseTime_ + MyDuration(1000) < MyDuration();
  }
  bool isFalling(bool swState) {
    return swState == false and prevSwState_ == true;
  }
  void sendScene_(uint8_t scene) {
    sceneMsg_.send(scene);
    #ifdef MY_MY_DEBUG
    Serial.print("sendScene_ ");
    Serial.print(scene);
    Serial.print(" for child id ");
    Serial.println(sceneMsg_.getMyMessage().sensor);
    #endif
  }
  void update_() override {
    bool currSwState = sw_.update();
    if (isRising_(currSwState)) {
      lastSwRiseTime_ = MyDuration();
      state_ = WAITING_FOR_SCENE;
    }
    else if (state_ == WAITING_FOR_SCENE and isHeldLongEnough_(currSwState)) {
      state_ = WAITING_FOR_FALLING;
      sendScene_(1);
    }
    else if (isFalling(currSwState)) {
      if (state_ == WAITING_FOR_SCENE) {
        if (enableShortPress_) {
          sendScene_(0);
        }
      }
      state_ = WAITING_FOR_RISING;
    }
    prevSwState_ = currSwState;
  }
public:
  MySceneController(uint8_t sensorId, Switch &sw, bool enableShortPress=true)
    : MyMySensorsBase(sensorId, S_SCENE_CONTROLLER),
      state_(WAITING_FOR_RISING),
      prevSwState_(false),
      sw_(sw),
      sceneMsg_(sensorId, V_SCENE_ON),
      enableShortPress_(enableShortPress) {}
};

template <typename ValueType, ValueType (*ReadValueCb)(), int16_t (*StartMeasurementCb)()>
class MyRequestingValue : public EventBase, public MyMySensorsBase
{
  MyMyMessage msg_;
  uint8_t childId_;
  uint8_t sensorType_;
  ValueType value_;
  int32_t interval_;
  void scheduleEvent(boolean (*cb)(EventBase*), int32_t delayMs)
  {
    this->period = 0;
    this->repeatCount = 1;
    this->nextTriggerTime = trueMillis() + delayMs;
    this->callback = cb;
    this->addEvent(this);
  }
  static boolean readValue_(EventBase* event)
  {
    MyRequestingValue* myRequestingValue = static_cast<MyRequestingValue*>(event);
    myRequestingValue->value_ = (*ReadValueCb)();
    #ifdef MY_MY_DEBUG
    Serial.print("readValue: ");
    Serial.print(myRequestingValue->value_);
    Serial.print(" next measurement: ");
    Serial.println(myRequestingValue->interval_);
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
  void begin_() {
    scheduleEvent(startMeasurement_, 0);
  }
  void receive_(const MyMessage &message) {
    if (message.type == sensorType_ and mGetCommand(message) == C_REQ)
      msg_.send(value_);
  }

public:
  MyRequestingValue(uint8_t sensorId, uint8_t type, uint8_t sensorType, int32_t interval)
    : MyMySensorsBase(sensorId, sensorType),
      msg_(sensorId, type),
      interval_(interval)
  {}
};

OneWire oneWire(TEMP_PIN);
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

#ifdef USE_APDS9930
MyAPDS9930 myApds(APDS9930_INT);
#endif

#ifdef KITCHEN
#define DIMMER1
#define DIMMER2
BounceSwitch sw1(A1, 50, true);
BounceSwitch sw2(A2, 50, true);
BounceSwitch sw3(A3, 50, true);
CwWwDimmer dim1(3, 5, true, 10, {1, 1});
SimpleDimmer dim2(6, false, 10, {1, 1});
MyRelaySwitch relay3(2, sw3, 10);
#endif

#ifdef LIVINGROOM
#define DIMMER1
#define DIMMER2
#define DIMMER3
AnalogBounceSwitch sw1(A7, 50, true);
BounceSwitch sw2(A2, 50, true);
BounceSwitch sw3(APDS9930_INT, 50, true);
CwWwDimmer dim1(3, 5, true, 10, {1, 1});
CwWwDimmer dim2(9, 10, false, 10, {1, 1});
SimpleDimmer dim3(6, true, 10, {0, 0});
#endif

#ifdef BATHROOM1
#define DIMMER1
#define SCENE2
#define SCENE2_ENABLE_SHORT true
#define SCENE3
#define SCENE3_ENABLE_SHORT true
BounceSwitch sw1(A3, 50, true);
BounceSwitch sw2(A2, 50, true);
BounceSwitch sw3(A1, 50, true);
SimpleDimmer dim1(10, false, 10, {1, 1});
#endif

#ifdef BATHROOM2
#define DIMMER1
#define SCENE2
#define SCENE2_ENABLE_SHORT true
#define SCENE3
#define SCENE3_ENABLE_SHORT true
BounceSwitch sw1(A3, 50, true);
BounceSwitch sw2(A2, 50, true);
BounceSwitch sw3(A1, 50, true);
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
SimpleDimmer dim1(3, true, 10, {0, 0});
SimpleDimmer dim2(5, true, 10, {0, 0});
#endif

#ifdef DIMMER1
MyDimmerSwitch dimmer1(0, dim1, sw1);
#endif
#ifdef DIMMER2
MyDimmerSwitch dimmer2(1, dim2, sw2);
#endif
#ifdef DIMMER3
MyDimmerSwitch dimmer3(2, dim3, sw3);
#endif
#ifdef SCENE1
MySceneController scene1(3, sw1, SCENE1_ENABLE_SHORT);
#endif
#ifdef SCENE2
MySceneController scene2(4, sw2, SCENE2_ENABLE_SHORT);
#endif
#ifdef SCENE3
MySceneController scene3(5, sw3, SCENE3_ENABLE_SHORT);
#endif

MyRequestingValue<float, readTempMeasurement, startTempMeasurement> temperature(6, V_TEMP, S_TEMP, 60000);

/***
 * Dimmable LED initialization method
 */
void setup()
{
  setClockPrescaler(CLOCK_PRESCALER_2);
  Serial.begin(115200);
  setPwmFrequency(3, 64); //488Hz, also pin 11
  setPwmFrequency(5, 64); //977Hz, also pin 6
  setPwmFrequency(9, 64); //488Hz, also pin 10
  tempSensor.begin();
  tempSensor.setWaitForConversion(false);
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

