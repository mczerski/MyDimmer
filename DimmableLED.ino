// Enable debug prints to serial monitor
//#define MY_DEBUG
//#define MY_DEBUG2

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_RF24_CE_PIN 7
#define MY_RF24_CS_PIN 4
#define MY_RF24_PA_LEVEL RF24_PA_MAX
#define MY_RF24_CHANNEL 100
//#define MY_RADIO_RFM69
#define MY_REPEATER_FEATURE
#define MY_OTA_FIRMWARE_FEATURE

#define MY_NODE_ID 6

#include <MySensors.h>
#include <Bounce2.h>

#define SKETCH_NAME "Dimmer"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "1"

MyMessage dimmerMsg(0, V_DIMMER);
MyMessage lightMsg(0, V_LIGHT);

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
      TCCR0B = TCCR0B & (0b11111000 | mode);
    } else {
      TCCR1B = TCCR1B & (0b11111000 | mode);
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
    TCCR2B = TCCR2B & (0b11111000 | mode);
  }
}

class Dimmer {
  enum State {
    OFF,
    ON,
    DIMMING_UP,
    DIMMING_DOWN
  };
  uint8_t pin_;
  bool lastPinValue_;
  State state_;
  uint8_t currentLevel_;
  uint8_t requestedLevel_;
  unsigned long nextChangeTime_;
  bool inverted_;

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
    //else if (currentLevel < 200)
    //  return 3;
    else
      return 3;
  }

  void setLevel_() {
    analogWrite(pin_, inverted_ ? 255 - currentLevel_ : currentLevel_);
  }

  bool updateLevel_() {
    if (state_ == OFF || state_ == ON)
      return false;
    unsigned long currentTime = millis();
    if (currentTime < nextChangeTime_)
      return false;
    if (state_ == DIMMING_DOWN && currentLevel_ > requestedLevel_) {
      currentLevel_--;
    }
    else if (state_ == DIMMING_UP && currentLevel_ < requestedLevel_) {
      currentLevel_++;
    }
    setLevel_();
    if (state_ == DIMMING_UP && currentLevel_ == requestedLevel_) {
      state_ = ON;
      return true;
    }
    else if (state_ == DIMMING_DOWN && currentLevel_ == requestedLevel_) {
      state_ = OFF;
      return true;
    }
    else {
      nextChangeTime_ = nextChangeTime_ + fadeDelay_();
      return false;
    }
  }

public:
  Dimmer(uint8_t pin, bool inverted = false, int pwmDivisor = 0)
    : pin_(pin), lastPinValue_(inverted),
      state_(OFF), currentLevel_(0),
      requestedLevel_(0), nextChangeTime_(0),
      inverted_(inverted) {
    setLevel_();
    if (pwmDivisor > 0)
      setPwmFrequency(pin, pwmDivisor);
  }

  bool update(bool value) {
    if (value == true && lastPinValue_ == false) {
      nextChangeTime_ = millis();
      if (state_ == OFF || state_ == DIMMING_DOWN) {
        requestedLevel_ = 255;
        state_ = DIMMING_UP;
      }
      else if (state_ == ON || state_ == DIMMING_UP) {
        requestedLevel_ = 0;
        state_ = DIMMING_DOWN;
      }
    }
    lastPinValue_ = value;
    return updateLevel_();
  }
  void request(uint8_t value) {
    if (value < currentLevel_) {
      state_ = DIMMING_DOWN;
    }
    else {
      state_ = DIMMING_UP;
    }
    nextChangeTime_ = millis();
    requestedLevel_ = value;
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
  }
  bool update() {
    switch_.update();
    return activeLow_ ? switch_.read() : !switch_.read();
  }
};

Dimmer dim1(3, true, 64);
Switch sw1(A1, 50, true);

/***
 * Dimmable LED initialization method
 */
void setup()
{
  Serial.begin(115200);
  // Pull the gateway's current dim level - restore light level upon sendor node power-up
  request( 0, V_DIMMER );
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

  // Register the LED Dimmable Light with the gateway
  present( 0, S_DIMMER );
}

/***
 *  Dimmable LED main processing loop 
 */
void loop()
{
  if (dim1.update(sw1.update()))
    sendCurrentLevel(dim1.getLevel());
}

void receive(const MyMessage &message) {
  if (message.isAck())
    return;
  if (message.type == V_LIGHT || message.type == V_DIMMER) {
    
    //  Retrieve the power or dim level from the incoming request message
    int requestedLevel = atoi( message.data );
    
    // Adjust incoming level if this is a V_LIGHT variable update [0 == off, 1 == on]
    requestedLevel *= ( message.type == V_LIGHT ? 100 : 1 );
    
    // Clip incoming level to valid range of 0 to 100
    requestedLevel = requestedLevel > 100 ? 100 : requestedLevel;
    requestedLevel = requestedLevel < 0   ? 0   : requestedLevel;

    Serial.print( "Changing level to " );
    Serial.print( requestedLevel );
    Serial.print( ", from " ); 
    Serial.println( dim1.getLevel() );

    requestedLevel = int(round(255.0*requestedLevel/100));
    dim1.request(requestedLevel);
  }
}

void sendCurrentLevel(uint8_t currentLevel) {
  uint8_t level = uint8_t(round(100.0 * currentLevel / 255));
  // Inform the gateway of the current DimmableLED's SwitchPower1 and LoadLevelStatus value...
  for (int i=0; i<10; i++)
    if (send(lightMsg.set(level > 0 ? 1 : 0), true))
      break;

  // hek comment: Is this really nessesary?
  for (int i=0; i<10; i++)
    if (send(dimmerMsg.set(level), true))
      break;
}

