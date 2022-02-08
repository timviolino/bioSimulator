#ifndef LABEL_H
#define LABEL_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>
#include "GUIBase.h"


class Label : public GUIBase {
  
  private:
    Adafruit_HX8357 tft;
    const String text;
    const int16_t y; 
    int16_t size;
    
    int16_t x;
    volatile uint16_t color = COLORS[TEXT];
    int16_t  x1, y1;
    uint16_t wi, he;
    
  
  public:
    Label(Adafruit_HX8357 &tft, const String text, const int16_t y, uint16_t size) ;
    
    void center();
    void draw(bool centered);
};

#endif
