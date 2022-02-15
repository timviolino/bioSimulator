#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>
#include "GUIBase.h"


class Button : public GUIBase {
  
  private:
   
    int16_t x;
    volatile uint16_t textColor = COLORS[BACKGROUND];
    volatile bool _state_fill = false;
    const uint16_t textSize = TEXT_SIZES[TITLES]; 
    
  
  public:
    Button();
    Adafruit_HX8357 tft = Adafruit_HX8357(9, 10, -1);
    String text;
    uint16_t y, fill;
    void select();
    void deselect();
    void draw();
    void erase();
};

#endif
