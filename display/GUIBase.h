#ifndef GUIBASE_H
#define GUIBASE_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>

class GUIBase 
{
  protected:
    GUIBase();
    const uint16_t COLORS[3] = {HX8357_BLACK, HX8357_GREEN, HX8357_WHITE};
    enum {BACK, BUTTONS, TEXT};
    const uint16_t WIDTH = 320;
    const uint8_t MARGIN = 5;
    int16_t  x1, y1;

    uint64_t get_del_t(uint64_t);

  public:
    int16_t x, y;
    uint16_t wi, he;
    Adafruit_HX8357 tft = Adafruit_HX8357(9, 10, -1);
};

#endif
