#ifndef CUBE_MARS_AK_h_
#define CUBE_MARS_AK_h_

#include <Arduino.h>

class CubeMarsAK
{
  private:
    unsigned int float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);

  public:
    CubeMarsAK();
    void enterMotorMode(uint8_t id);
    void exitMotorMode(uint8_t id);
    void zero(uint8_t id);
};


#endif
