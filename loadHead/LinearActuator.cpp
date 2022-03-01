#include "LinearActuator.h"

LinearActuator::LinearActuator(uint8_t RPWM, uint8_t LPWM)
  : _RPWM(RPWM)
  , _LPWM(LPWM)
  {}

void LinearActuator::setSpeed(uint8_t v, bool sign)
{
  const uint8_t pinHI = (sign) ? _RPWM : _LPWM;             // set _RPWM HI if negative
  const uint8_t pinLO = (pinHI == _RPWM) ? _LPWM : _RPWM;   // set opposite LO
  analogWrite(pinHI, v);                      
  analogWrite(pinLO, 0);                      
}

void LinearActuator::step(int16_t v, uint16_t t)
{
  const bool sign = signbit(v);     // save 1 if neg, 0 if pos
  setSpeed(abs(v), sign);           // set speed [0-255], retract if 1, extend if 0
  delay(t);                         // delay in millis [0-65536]
  setSpeed(0, sign);                // set speed to 0
}

void LinearActuator::init() {
  pinMode(_RPWM, OUTPUT);        // configure pin 10 as an output
  pinMode(_LPWM, OUTPUT);        // configure pin 11 as an output
  step(-80, 1000);               // retract for 2 sec
  step(80, 900);                 // extend for 1 sec
}
