#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H

#include <Arduino.h>

class LinearActuator
{
  private:
    uint8_t _RPWM, _LPWM;
    
  public:
    LinearActuator(uint8_t RPWM, uint8_t LPWM);
    void setSpeed(int8_t v);
    void retract(uint64_t);
    void extend(uint64_t);
    void init();
};

#endif
