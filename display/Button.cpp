#include "Button.h"

Button::Button(Adafruit_HX8357 &tft, const String text, const int16_t y, const uint16_t fillColor) 
  : GUIBase()
  , tft(tft)
  , text(text)
  , y(y)
  , fillColor(fillColor)
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
  Serial.println("Text = " + String(text) + " x = " + String(x) + " y = " + String(y) + " wi = " + String(wi) + " he = " + String(he));
  tft.getTextBounds(text, 0, y, &x1, &y1, &wi, &he);
  x = (WIDTH-wi)/2;                                         
  if(!backgroundPrinted) {
    tft.fillRoundRect(x-5, y-5, wi+5, he+5, 5, fillColor);
    backgroundPrinted = true;
  }
  tft.setCursor(x, y);
  tft.println(text);
 
}
