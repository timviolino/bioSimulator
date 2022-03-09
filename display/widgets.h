#ifndef WIDGETS_H
#define WIDGETS_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>    // https://github.com/adafruit/Adafruit_HX8357_Library

enum {OPTIONS, LABELS, TITLES};                         // indices of text size array
enum {BACK, BUTTONS, TEXT};                             // indices of colors array
const uint8_t SIZES[3] = {2, 3, 4};
const uint16_t COLORS[3] = {HX8357_BLACK, HX8357_GREEN, HX8357_WHITE};
const uint8_t MARGIN = 5;
const uint16_t WIDTH = HX8357_TFTWIDTH-2*MARGIN;
const uint16_t HEIGHT = HX8357_TFTHEIGHT-2*MARGIN;

class GUIBase 
{
  protected:
    GUIBase();
    int16_t  x1, y1;
    uint64_t get_del_t(uint64_t);

  public:
    int16_t x, y;
    uint16_t wi, he;
    Adafruit_HX8357 tft = Adafruit_HX8357(9, 10, -1);
};

class Label : public GUIBase {
  
  protected:
    const uint16_t TEXT_SIZES[3] = {2, 3, 4};
    enum {OPTIONS, LABELS, TITLES};
  
  public:
    char *text;
    int16_t size;
    uint16_t color = COLORS[TEXT];
    bool centered = false;
  
    Label();
    void center();
    void draw();
    void changeSize(int8_t);
};

class Button : public Label {
  
  private:
    volatile bool filled = false;
    void setFill();
  
  public:
    Button();
    uint16_t fill = COLORS[BUTTONS];
    void select(bool);
    void draw();
    void erase();
};

class ProgressBar : public GUIBase {

  private:
    uint16_t _wi_in;
    const uint8_t _R = 10;
    const uint32_t HRS_TO_S = 60*60;
    const uint32_t HRS_TO_MS = HRS_TO_S*1000;
    float _getPercent();

  public:
    enum {OUT, IN};
    bool complete = false;
    uint16_t fill[2] = {HX8357_WHITE, HX8357_GREEN};
    uint16_t duration = 10;                              // 10x number of hours 
    uint64_t t_start = 0;
    ProgressBar();
    void draw();
  
};
#endif
