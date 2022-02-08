#include "controlKnob.h"

controlKnob::controlKnob(const byte PIN_MSB, const byte PIN_LSB, const byte PIN_BUTTON) 
  : PIN_MSB(PIN_MSB)
  , PIN_LSB(PIN_LSB)
  , PIN_BUTTON(PIN_BUTTON)
  {}


  int8_t controlKnob::readEncoder()
  {
    encoderPos = 0;
    MSB1 = digitalRead(2); LSB1 = digitalRead(3);                         // read encoder values [0, 1]
    if ((MSB1 == 1 && LSB1 == 1) && (MSB0 == 1 && LSB0 == 0)) {           // encoder readEncodered to the right
      encoderPos = -1;                                                    // increment encoder position
    }
    else if ((MSB1 == 1 && LSB1 == 1) && (MSB0 == 0 && LSB0 == 1)) {      // encoder readEncodered to the left
      encoderPos = 1;                                                     // decrement encoder position
    }
    MSB0 = MSB1; LSB0 = LSB1;                                             // record encoder values for next check
    return encoderPos;
  }

  int16_t controlKnob::getButtonPush() 
  {
    isPushed = false;
    btn1 = digitalRead(PIN_BUTTON);       // read button state [LOW, HIGH]
    t1 = millis();                        // save the time [ms]
    del_t = t1-t0;
    if(btn1 != btn0 && (del_t > 50))      // check if the button state has changed in a realistic time frame
    {
      t0 = t1;                            // save the time that the button was pushed [ms]
      btn0 = btn1;                        // save the current state of the button [LOW, HIGH]
      if(btn1 == LOW){isPushed = true;}   // LOW means that the button has been pushed => do push code
    }
    return isPushed;
  }

  void controlKnob::attach() 
  {
    pinMode(PIN_MSB, INPUT_PULLUP);
    pinMode(PIN_LSB, INPUT_PULLUP);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
  }
