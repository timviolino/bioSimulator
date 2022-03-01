#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H

#include <Arduino.h>

class LinearActuator
{
  private:
    uint8_t _RPWM, _LPWM;
    
  public:
    LinearActuator(uint8_t RPWM, uint8_t LPWM);
    void setSpeed(uint8_t, bool);
    void step(int16_t v, uint16_t t);           // v < 0 => RETRACT, v > 0 => EXTEND, t in ms
    void init();
};

#endif
