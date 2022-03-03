#include "widgets.h"

Label::Label() 
  : GUIBase()
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

Button::Button() 
  : Label()
{}

void Button::select() {color = COLORS[TEXT];}

void Button::deselect() {color = COLORS[BACKGROUND];}

void Button::draw() 
{                                    
  if(!filled) {setFill();}
  Label::draw(true);
}

void Button::setFill()
{
  tft.setTextSize(size);
  center();
  tft.fillRoundRect(x-5, y-5, wi+5, he+5, 5, fill);
  filled = true;
}

void Button::erase()
{
  deselect();
  filled = false;
}
