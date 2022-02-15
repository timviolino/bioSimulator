#ifndef controlKnob_H
#define controlKnob_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>


class controlKnob {
  
  private:
    const byte PIN_MSB, PIN_LSB, PIN_BUTTON;
    int MSB0, MSB1, LSB0, LSB1;
    uint64_t t1, del_t;
    uint64_t t0 = 0;
    bool btn0 = HIGH;
    bool btn1;
    bool isPushed = false;
  
  public:
    controlKnob(const byte PIN_MSB, const byte PIN_LSB, const byte PIN_BUTTON);

    int8_t encoderPos;
    int8_t readEncoder();
    int16_t getButtonPush();
    void attach();
};

#endif
