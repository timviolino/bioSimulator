#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>
#include "GUIBase.h"


class Button : public GUIBase {
  
  private:
    Adafruit_HX8357 tft;
    const String text;
    const int16_t y; 
    const uint16_t fillColor;
    
    int16_t x;
    volatile uint16_t textColor = COLORS[BACKGROUND];
    volatile bool backgroundPrinted = false;
    const uint16_t textSize = TEXT_SIZES[TITLES]; 
    
  
  public:
    Button(Adafruit_HX8357 &tft, const String text, const int16_t y, const uint16_t fillColor) ;
    
    void select();
    void deselect();
    void draw();
};

#endif
