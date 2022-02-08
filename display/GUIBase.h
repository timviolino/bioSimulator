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
    enum {BACKGROUND, BUTTONS, TEXT};
    
    const uint16_t TEXT_SIZES[3] = {2, 3, 4};
    enum {OPTIONS, LABELS, TITLES};

    const uint16_t WIDTH = 320;
};

#endif
