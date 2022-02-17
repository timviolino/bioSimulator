#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H

#include <Arduino.h>

const uint64_t T_RETRACT = 3000;

class LinearActuator
{
  private:
    uint8_t _RPWM, _LPWM;
    
  public:
    LinearActuator(uint8_t RPWM, uint8_t LPWM);
    void setSpeed(int8_t v);
    void retract();
    void init();
};

#endif
