////////////////////////////////////////// Library Imports //////////////////////////////////////////
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>            // https://github.com/adafruit/Adafruit_HX8357_Library
#include <Wire.h>                       // Used for networking between Arduinos
#include "controlKnob.h"
#include "widgets.h"

///////////////////// Constant Value Definitions /////////////////////
#define TFT_RST -1
#define KNOB_MSB 2
#define KNOB_LSB 3
#define KNOB_BUTTON 4
#define TFT_DC 9
#define TFT_CS 10
enum {BACKGROUND, BUTTONS, TEXT};                                     // indices of graphics colors array
enum {OPTIONS, LABELS, TITLES};                                       // indices of text size array
enum {PARAM, VALUE, UNIT};                                            // cols of label array
enum {FREQUENCY, STROKE, LOAD, HOURS, UPLOAD_BUTTON};                 // indices of cursor position
enum {BOOT, USER_INPUT, UPLOAD, RUN_TEST, COMPLETE};                  // indices of machine states
enum {BTN_UPLOAD, BTN_START, BTN_STOP, BTN_RESTART};                  // indices of btn objects
enum {UI, STAGE, LOAD_HEAD};                                          // indices of arduino addresses
enum {START, STOP};
const uint8_t INO_ADDYS[3] = {9, 10, 11};
const uint8_t CODES[2] = {245, 255};   // i2c codes
const uint8_t PARAM_DESTINATION[3] = {INO_ADDYS[STAGE], INO_ADDYS[STAGE], INO_ADDYS[LOAD_HEAD]};
const uint8_t N_PARAMS = 4;
const uint8_t N_BTNS = 4;
const uint8_t N_OPTIONS = 5;
const uint8_t S_TEXT[3] = {2, 3, 4};
const uint8_t S_LABEL[3] = {S_TEXT[OPTIONS], S_TEXT[OPTIONS], S_TEXT[OPTIONS]};
const uint8_t PARAMS_CHARS[N_PARAMS] = {3, 3, 4, 3};
const uint8_t PARAMS_MIN[N_PARAMS] = {1, 3, 50, 1};
const float PARAMS_MAX[N_PARAMS] = {5.00, 7.50, 400.0, 744.0};
const float PARAMS_STEP[N_PARAMS] = {0.1, 0.1, 10, 1};
const uint16_t COLORS[3] = {HX8357_BLACK, HX8357_GREEN, HX8357_WHITE};
const uint16_t WIDTH = HX8357_TFTWIDTH;
const uint16_t HEIGHT = HX8357_TFTHEIGHT;
const uint8_t MARGIN = 5;
const uint8_t ROWS = N_OPTIONS + 1;
const uint8_t COLS = 3;
const uint16_t ROW_HEIGHT = (HEIGHT-2*MARGIN)/(ROWS);
const uint16_t COL_WIDTH = (WIDTH-2*MARGIN)/(COLS);
const uint16_t COL_WIDTHS[3] = {(WIDTH-2*MARGIN)/2, (WIDTH-2*MARGIN)/3, (WIDTH-2*MARGIN)/6};
const char BTN_TEXT[N_BTNS][11] = {"Upload", "Start Test", "Stop Test", "Restart"};
const uint16_t BTN_Y = MARGIN+(N_OPTIONS)*ROW_HEIGHT;
const uint16_t BTN_FILLS[N_BTNS] = {HX8357_GREEN, HX8357_GREEN, HX8357_RED, HX8357_GREEN};
const uint64_t BAUD_RATE = 115200; 
const uint64_t BLINK_TIME = 500;                                        // time [ms] of cursor blink

////////////////////////////////////////// Global Variable Declarations //////////////////////////////////////////
bool state_enc = true;                                      // selects options if true, selects values if false
bool state_cursor = false;                                  // true: cursor has been printed | false: cursor has not been printed
uint8_t state = BOOT;                                       // machine state
uint8_t i_cursor = FREQUENCY;                               // index of the cursor
int8_t i_enc;                                               // number of encoder steps since last interrupt
uint64_t t_start;                                           // time [ms] at which program begins
uint64_t t_lastBlink = 0.0;                                 // time [ms] that cursor last blinked
volatile float params[N_PARAMS] = {1.0, 3.0, 50.0, 1.0};    // stores the test parameters as user updates them
char labels[N_PARAMS][3][12] = 
{
  {"Frequency: ", "1.0", "Hz"}, 
  {"Stroke: ", "3.0", "Deg"},
  {"Load: ", "50.0", "N"},
  {"Duration: ", "1.0", "hr"}
};

///////////////////// Object Declarations /////////////////////
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);
controlKnob knob = controlKnob(KNOB_MSB, KNOB_LSB, KNOB_BUTTON);
Button btn;
Label title;
void(* resetFunc) (void) = 0;

void setup() 
{
  mySerialBegin();
  initTFT();
  initControlKnob();
  initTitle();
  initBtn();
  Wire.begin(9);
}

void loop() 
{
  myReadButton();
  switch(state) 
  {
    case BOOT:
      title.draw(true);
      delay(1000);
      nextState();
    break;
    
    case USER_INPUT:
      updateCursor();
      myPrint("Parameters", 0, MARGIN, S_TEXT[TITLES], COLORS[TEXT], true);
      printOptions();
      btn.draw();
    break;
    
    case UPLOAD:
    myPrint("Parameters Uploaded", 0, MARGIN + 2*ROW_HEIGHT, S_TEXT[OPTIONS], COLORS[TEXT], true);
    myPrint("Mount specimen now!", 0, MARGIN + 3*ROW_HEIGHT, S_TEXT[OPTIONS], COLORS[TEXT], true);
    break;

    case RUN_TEST:
    myPrint("Test in Progress...", 0, MARGIN + 2*ROW_HEIGHT, S_TEXT[OPTIONS], COLORS[TEXT], true);
    printProgressBar();
    break;

    case COMPLETE:
    myPrint("Test Completed", 0, MARGIN + 2*ROW_HEIGHT, S_TEXT[OPTIONS], COLORS[TEXT], true);
    myPrint("Please remove specimen", 0, MARGIN + 3*ROW_HEIGHT, S_TEXT[OPTIONS], COLORS[TEXT], true);
    break;   
  }
}

void initTFT() 
{
  delay(100);
  tft.begin();
  tft.setRotation(2);
  tft.setTextWrap(false);
  tft.fillScreen(COLORS[BACKGROUND]);
}

void initControlKnob() 
{
   knob.attach();
   attachInterrupt(0, readKnob, CHANGE);
   attachInterrupt(1, readKnob, CHANGE);
}

void initTitle()
{
  title.text = "bioSimulator";
  title.y = MARGIN + 2*ROW_HEIGHT;
  title.size = S_TEXT[TITLES];
  title.tft = tft;
}

void initBtn()
{
  btn.tft = tft;
  btn.y = BTN_Y;
}

void updateBtn(uint8_t i) 
{
  btn.text = BTN_TEXT[i];
  btn.fill = BTN_FILLS[i];
}

void nextState() 
{
  state++;
  tft.fillScreen(COLORS[BACKGROUND]);
  btn.erase();
  btn.select();
  updateBtn(state-1);
  switch(state) 
  {
    case USER_INPUT:
      i_cursor = 0;
      moveCursor(i_cursor, i_cursor);
    break;
    case RUN_TEST:
      t_start = millis();
    break;
    case COMPLETE:
      sendInoSignal(CODES[STOP]);
    break;
  }
  btn.draw();
}

///////////////////// Control Knob Helpers /////////////////////
// NOTE this is not an interrupt because both external interrupt pins are used for the encoder
void myReadButton() {if(knob.getButtonPush()) {push();}}

void readKnob() {i_enc += knob.readEncoder();}

void push() 
{
  switch(state) 
  {
    case USER_INPUT:
      if (i_cursor == UPLOAD_BUTTON) 
      {
        nextState();
        uploadParams();
        break;
      }
      else 
      {
        state_enc = !state_enc;
        moveCursor(i_cursor, i_cursor);
      }
    break;
    case UPLOAD:
      nextState();
      sendInoSignal(CODES[START]);
    break;
    case RUN_TEST:
      state = BOOT;
      sendInoSignal(CODES[STOP]);
      nextState();
    break;
    case COMPLETE:
      resetFunc();
    break;
  }
}

void updateCursor() 
{
  if (state_enc) 
  {
    if (i_enc != 0) 
    {
      uint8_t i_cursor_last = i_cursor;
      i_cursor = wrap(i_cursor+i_enc, FREQUENCY, UPLOAD_BUTTON);
      moveCursor(i_cursor_last, i_cursor);
    }
  }
  else
  {
    if (i_enc != 0) {
      updateParam(i_cursor, i_enc);
      moveCursor(i_cursor, i_cursor);
      t_lastBlink = millis();
    }
    blinkCursor(i_cursor);
  }
  i_enc = 0;
}

void updateParam(uint16_t i, int8_t x) 
{
  params[i] += x*PARAMS_STEP[i];
  params[i] = constrain(params[i], PARAMS_MIN[i], PARAMS_MAX[i]);
  dtostrf(params[i], PARAMS_CHARS[i], 1, labels[i][1]);
}

void uploadParams()
{
  uint8_t param;                                       // stores the param to be transmitted
  const float factor[3] = {10.0, 10.0, 0.1};           // used to retain decimals when converting to uint8_t
  for (uint8_t i = 0; i < N_PARAMS-1; i++) 
  {
    Wire.beginTransmission(PARAM_DESTINATION[i]);
    param = (uint8_t) (params[i]*factor[i]);
    Wire.write(param);
    Wire.endTransmission();
  }
}

void sendInoSignal(uint8_t code)
{
  for(uint8_t i = 0; i < 2; i++)
  {
    Wire.beginTransmission(INO_ADDYS[i+1]);
    Wire.write(code);
    Wire.endTransmission();
    delay(50);
  }
}

///////////////////// Graphics Helpers /////////////////////
void printProgressBar()
{
  const int16_t y = MARGIN+(3)*ROW_HEIGHT;
  const uint16_t f_0 = HX8357_WHITE;
  const uint16_t f_1 = HX8357_GREEN;
  const uint16_t w0 = WIDTH-10*MARGIN; 
  const uint16_t h = ROW_HEIGHT;
  const uint16_t r = 10;
  const int16_t x = (WIDTH-w0)/2;
  float del_t = get_del_t(t_start);
  float percent = del_t/(params[HOURS]*3600000.f);
  const uint16_t w1 = (w0-r)*percent;
  tft.drawRoundRect(x, y, w0, h, r, f_0);                           // print border
  tft.fillRoundRect(x+1, y+1, 2*r, h-2, r, f_1);                    // need this to prevent curves from overlapping
  if (w1 < w0-r) {tft.fillRoundRect(x+r, y+1, w1, h-2, r, f_1);}    // print and update filler
  else {nextState();}
}

void moveCursor(uint16_t p0, uint16_t p1) 
{
  String label;
  uint16_t x = MARGIN + COL_WIDTHS[0];
  uint16_t y;
  
  // erase last cursor
  btn.deselect();
  if (p0 != UPLOAD_BUTTON)
  {
    label = labels[p0][VALUE]; 
    y = MARGIN + (p0+1)*ROW_HEIGHT;
    erase("000.0", x, y, S_TEXT[LABELS]);
  }
  
  // print next cursor
  if (p1 != UPLOAD_BUTTON) 
  {
    label = labels[p1][VALUE];
    y = MARGIN + (p1+1)*ROW_HEIGHT;
    erase(label, x, y, S_TEXT[OPTIONS]);
    myPrint(label, x, y, S_TEXT[LABELS], COLORS[TEXT], false);
  }
  else {btn.select();}
  
  state_cursor = true;
}

void blinkCursor(uint16_t p) 
{
  String label = labels[p][VALUE];
  int x = MARGIN + COL_WIDTHS[0];
  int y = MARGIN + (p+1)*ROW_HEIGHT;
  uint16_t textSize = S_TEXT[LABELS];
  unsigned long del_t = get_del_t(t_lastBlink);
   
  if (del_t < BLINK_TIME) {
    myPrint(label, x, y, textSize, COLORS[TEXT], false);
  }
  else if (del_t < 2*BLINK_TIME) {
    erase(label, x, y, textSize);
  }
  else {t_lastBlink += del_t;}
}

void printOptions() 
{
  int x, y;
  String label;
  
  // print params
  for(uint16_t i = 0; i < N_PARAMS; i++) 
  {
    x = MARGIN; y = MARGIN + (i+1)*ROW_HEIGHT;
    for (uint16_t j = 0; j < COLS; j++) 
    {
      label = labels[i][j];
      if (i_cursor != i || j != VALUE){
        myPrint(label, x, y, S_LABEL[j], COLORS[TEXT], false);
      }
      x += COL_WIDTHS[j];
    }
  }
}

void erase(String label, int16_t x, int16_t y, uint16_t textSize)
{
  int16_t  x1, y1;
  uint16_t wi, he;
  
  tft.setTextSize(textSize);
  tft.getTextBounds(label, x, y, &x1, &y1, &wi, &he);
  tft.fillRect(x, y, wi, he, COLORS[BACKGROUND]);
}

void myPrint(String label, int16_t x, int16_t y, uint16_t textSize, uint16_t textColor, bool centered) 
{
  int16_t  x1, y1;
  uint16_t wi, he;
    
  tft.setTextColor(textColor);
  tft.setTextSize(textSize);
  if (centered) 
  {
    tft.getTextBounds(label, 0, 0, &x1, &y1, &wi, &he);
    x = (WIDTH-wi)/2; // center aligns text
  }
  tft.setCursor(x, y);
  tft.println(label);
}

//////////////////////////// Generic Helper Functions ////////////////////////////
unsigned long get_del_t(unsigned long t0) {return millis()-t0;}

uint16_t wrap(int16_t x, int16_t X0, int16_t X1) 
{
  if (x > X1) {return X0;}
  else if (x < X0) {return X1;}
  return x;
}

void mySerialBegin() 
{
  Serial.begin(BAUD_RATE);
  while(!Serial);
}
