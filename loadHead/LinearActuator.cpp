#include "LinearActuator.h"

LinearActuator::LinearActuator(uint8_t RPWM, uint8_t LPWM)
  : _RPWM(RPWM)
  , _LPWM(LPWM)
  {}

void LinearActuator::setSpeed(int8_t v)
{
  analogWrite(_RPWM, 0);
  analogWrite(_LPWM, 0);
  if (v < 0) {analogWrite(_RPWM, abs(v));}    // Retract actuator
  else {analogWrite(_LPWM, abs(v));}          // Extend actuator
  delay(30);                                 // Minimum travel time
}

void LinearActuator::retract()
{
  setSpeed(-90);
  delay(T_RETRACT);
  setSpeed(0);
}

void LinearActuator::init() {
  pinMode(_RPWM, OUTPUT);        // configure pin 10 as an output
  pinMode(_LPWM, OUTPUT);        // configure pin 11 as an output
  retract();
}
