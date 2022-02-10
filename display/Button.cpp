#include "Button.h"

Button::Button(const String text, const int16_t y, const uint16_t fillColor) 
  : GUIBase()
  , text(text)
  , y(y)
  , fillColor(fillColor)
{}

void Button::setTFT(Adafruit_HX8357 tft) {
  _tft = tft;
}

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
  _tft.setTextSize(textSize);
  _tft.setTextColor(textColor);
  Serial.println("Text = " + String(text) + " x = " + String(x) + " y = " + String(y) + " wi = " + String(wi) + " he = " + String(he));
  _tft.getTextBounds(text, 0, y, &x1, &y1, &wi, &he);
  x = (WIDTH-wi)/2;                                         
  if(!backgroundPrinted) {
    _tft.fillRoundRect(x-5, y-5, wi+5, he+5, 5, fillColor);
    backgroundPrinted = true;
  }
  _tft.setCursor(x, y);
  _tft.println(text);
 
}
