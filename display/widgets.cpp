#include "widgets.h"

GUIBase::GUIBase()
{}

uint64_t GUIBase::get_del_t(uint64_t t0) {return millis()-t0;}

Label::Label() 
  : GUIBase()
{}

void Label::center()
{
  tft.getTextBounds(text, 0, y, &x1, &y1, &wi, &he);
  x = (WIDTH-wi)/2;
}

void Label::draw() 
{
  tft.setTextSize(size);
  if(centered){center();}
  tft.setTextColor(color);                                    
  tft.setCursor(x, y);
  tft.println(text);
}

void Label::changeSize(int8_t x)
{
  color = COLORS[BACK];
  draw();
  size += x;
  color = COLORS[TEXT];
  draw();
}

Button::Button() 
  : Label()
{}

void Button::select(bool selected) {color = (selected) ? COLORS[TEXT] : COLORS[BACK];}

void Button::draw() 
{                                    
  Serial.println(text);
  centered = true;
  if(!filled) {setFill();}
  Label::draw();
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
  select(false);
  filled = false;
}

ProgressBar::ProgressBar()
  :GUIBase()
{}

void ProgressBar::draw()
{
  const float percent = _getPercent();
  _wi_in = (wi-_R)*percent;
  complete = (wi < _wi_in+_R);
  tft.drawRoundRect(x, y, wi, he, _R, fill[OUT]);                               // print border
  tft.fillRoundRect(x+1, y+1, 2*_R, he-2, _R, fill[IN]);                        // need this to prevent curves from overlapping
  if (!complete) {tft.fillRoundRect(x+_R, y+1, _wi_in, he-2, _R, fill[IN]);}    // print and update filler
}

float ProgressBar::_getPercent()
{
  uint64_t del_t = get_del_t(t_start);
  return del_t/(duration/10.f*HRS_TO_MS);
}
