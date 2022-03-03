#ifndef WIDGETS_H
#define WIDGETS_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>
#include "GUIBase.h"


class Label : public GUIBase {
  
  protected:
    int16_t  x1, y1;
    uint16_t wi, he;
    uint16_t color = COLORS[TEXT];
  
  public:
    int16_t x, y;
    String text;
    int16_t size;
    Adafruit_HX8357 tft = Adafruit_HX8357(9, 10, -1);
  
    Label();
    void center();
    void draw(bool centered);
};

class Button : public Label {
  
  private:
    volatile bool filled = false;
    void setFill();
  
  public:
    Button();
    uint16_t fill = COLORS[BUTTONS];
    void select();
    void deselect();
    void draw();
    void erase();
};

#endif
