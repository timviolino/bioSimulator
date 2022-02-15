#include "Button.h"

Button::Button() 
  : GUIBase()
{}

void Button::select() {
  textColor = COLORS[TEXT];
}

void Button::deselect() {
  textColor = COLORS[BACKGROUND];
}

void Button::draw() 
{
  int16_t  x1, y1;
  uint16_t wi, he;
  tft.setTextSize(textSize);
  tft.setTextColor(textColor);
  tft.getTextBounds(text, 0, y, &x1, &y1, &wi, &he);
  x = (WIDTH-wi)/2;                                      
  if(!_state_fill) {
    tft.fillRoundRect(x-5, y-5, wi+5, he+5, 5, fill);
    _state_fill = true;
  }
  tft.setCursor(x, y);
  tft.println(text);
}

void Button::erase()
{
  deselect();
  _state_fill = false;
}
