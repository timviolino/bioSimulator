#include "Label.h"

Label::Label(Adafruit_HX8357 &tft, const String text, const int16_t y, uint16_t size) 
  : GUIBase()
  , tft(tft)
  , text(text)
  , y(y)
  , size(size)
{}

void Label::center()
{
  tft.getTextBounds(text, 0, y, &x1, &y1, &wi, &he);
  x = (WIDTH-wi)/2;
}

void Label::draw(bool centered) 
{
  tft.setTextSize(size);
  if(centered){center();}
  tft.setTextColor(color);                                    
  tft.setCursor(x, y);
  tft.println(text);
}
