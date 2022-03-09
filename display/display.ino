////////////////////////////////////////// Library Imports //////////////////////////////////////////
#include <SPI.h>        
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
enum {FREQUENCY, STROKE, LOAD, HOURS, BTN};             // indices of cursor position
enum {BOOT, USER_INPUT, UPLOAD, RUN_TEST, COMPLETE};    // indices of machine states
enum {STAGE, LH};                                       // indices of arduino addresses
enum {START, STOP};                                     // indices of CODES
const uint8_t INO_ADDYS[2] = {10, 11};
const uint8_t CODES[2] = {245, 255};                                  // i2c codes
const uint8_t N_PARAMS = 4;
const uint8_t N_BTNS = 4;            
const uint8_t ROWS = 6;
const uint8_t COLS = 3;
const uint16_t ROW_HEIGHT = HEIGHT/ROWS;
const uint16_t COL_WIDTH = WIDTH/COLS;
const uint16_t COL_WIDTHS[3] = {WIDTH/2, WIDTH/3, WIDTH/6};
const uint32_t BAUD_RATE = 115200; 
char TITLE_TEXT[7][23] = {"bioSimulator", "Parameters", "Parameters Uploaded", "Mount specimen now!", "Test in Progress...", "Test Completed", "Please remove specimen"};
char BTN_TEXT[N_BTNS][11] = {"Upload", "Start Test", "Stop Test", "Restart"};

////////////////////////////////////////// Global Variable Declarations //////////////////////////////////////////
bool blinking = false;                                      // whether or not parameter is being edited
bool blinkPrinted = false;
uint8_t state = BOOT;                                       // machine state
uint8_t i_cursor = FREQUENCY;                               // index of the cursor
int8_t i_enc;                                               // number of encoder steps since last interrupt
uint64_t t_lastBlink = 0;                                   // time [ms] that cursor last blinked
volatile uint16_t params[N_PARAMS] = {10, 30, 500, 10};     // stores 10x the test parameters as user updates them
uint16_t pb_delay;

///////////////////// Object Declarations /////////////////////
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);
controlKnob knob = controlKnob(KNOB_MSB, KNOB_LSB, KNOB_BUTTON);
Button btn;
Label title;
ProgressBar PB;
void(* resetFunc) (void) = 0;

void setup() 
{
  mySerialBegin();
  initTFT();
  initControlKnob();
  initLabel();
  initBtn();
  Wire.begin(9);
}

void loop() //all dynamic widgets happen here
{
  myReadButton();
  switch(state) 
  {
    case BOOT:
      updateLabel(TITLE_TEXT[0], MARGIN + 2*ROW_HEIGHT, SIZES[TITLES]);
      delay(1000);
      nextState();
    break;
    
    case USER_INPUT:
      updateCursor();
    break;
    
    case UPLOAD:
    break;

    case RUN_TEST:
      pb_delay = get_del_t(PB.t_start);
      if(pb_delay%1000 == 0)  {PB.draw();}        // print progress bar until test is done, then go to next state
      if(PB.complete) {nextState();}
    break;

    case COMPLETE:
    break;   
  }
}

void nextState() // state transfer function: all static widgets happen here
{
  const uint16_t he[2] = {MARGIN + 2*ROW_HEIGHT, MARGIN + 3*ROW_HEIGHT};
  const uint8_t s = SIZES[OPTIONS];
  state++;
  tft.fillScreen(COLORS[BACK]);
  switch(state) 
  {
    case USER_INPUT:
      updateLabel(TITLE_TEXT[1], MARGIN, SIZES[TITLES]);
      i_cursor = 0;
      drawOptions();
      initCursor();
    break;
    case UPLOAD:
      updateLabel(TITLE_TEXT[2], he[0], s);
      updateLabel(TITLE_TEXT[3], he[1], s);
    break;
    case RUN_TEST:
      updateLabel(TITLE_TEXT[4], he[0], s);
      sendInoSignal(CODES[START]);
      initProgressBar();
    break;
    case COMPLETE:
      updateLabel(TITLE_TEXT[5], he[0], s);
      updateLabel(TITLE_TEXT[6], he[1], s);
      sendInoSignal(CODES[STOP]);
    break;
  }
  updateBtn(state-1);
}

void initTFT() 
{
  delay(100);
  tft.begin();
  tft.setRotation(2);
  tft.setTextWrap(false);
  tft.fillScreen(COLORS[BACK]);
}

void initControlKnob() 
{
   knob.attach();
   attachInterrupt(0, readKnob, CHANGE);
   attachInterrupt(1, readKnob, CHANGE);
}

void initLabel()
{
  title.tft = tft;
}

void initBtn()
{
  btn.tft = tft;
  btn.y = MARGIN+(ROWS-1)*ROW_HEIGHT;
  btn.size = SIZES[TITLES];
}

void initProgressBar()
{
  PB.tft = tft;
  PB.wi = WIDTH-8*MARGIN;
  PB.he = ROW_HEIGHT;
  PB.x = (WIDTH+2*MARGIN-PB.wi)/2;
  PB.y = MARGIN+(3)*ROW_HEIGHT;
  PB.duration = params[HOURS];
  PB.t_start = millis();
  PB.draw();
}

void updateBtn(uint8_t i) 
{
  const bool BTN_FILLS[N_BTNS] = {0, 0, 1, 0};
  btn.erase();
  btn.text = BTN_TEXT[i];
  btn.fill = (BTN_FILLS[i] == 0) ? HX8357_GREEN : HX8357_RED;
  if (i+1 != USER_INPUT) {btn.select(true);}
  btn.draw();
}

void updateLabel(char* text, int16_t y, uint16_t size)
{
  title.text = text;
  title.y = y;
  title.size = size;
  title.centered = true;
  title.draw();
}

///////////////////// Control Knob Helpers /////////////////////
// NOTE this is not an interrupt because both external interrupt pins are used for the encoder
void myReadButton() {if(knob.getButtonPush()) {push();}}

void readKnob() {i_enc += knob.readEncoder();}

void push() 
{
  bool next = true;
  switch(state) 
  {
    case USER_INPUT:
      if (i_cursor == BTN) {uploadParams();}
      else 
      {
        next = false;
        blinking = !blinking;
        initCursor();
        t_lastBlink = millis();
      }
    break;
    case RUN_TEST:
      state = BOOT;
      sendInoSignal(CODES[STOP]);
    break;
    case COMPLETE:
      resetFunc();
    break;
  }
  if (next) {nextState();}
}

void initCursor()
{
  char label[6];
  getParamChar(i_cursor, label);
  drawCursor(i_cursor, i_cursor, label, label);
}

void updateCursor() 
{
  const uint8_t i0 = i_cursor;
  uint8_t i1 = i0;
  bool blinkChange = false;
  char label0[6], label1[6];
  if(blinking) {
    bool temp = getBlink();
    blinkChange = (temp != blinkPrinted);
    blinkPrinted = temp;
  }
  bool encChange = (i_enc != 0);
  if (encChange && blinking) {t_lastBlink = millis();}
  if (encChange || blinkChange) {
    if (i0 != BTN) {getParamChar(i0, label0);}
    if (blinking) {updateParam(i_cursor, i_enc);}
    else {i1 = wrap(i_cursor+i_enc, FREQUENCY, BTN);}
    if (i1 != BTN) {getParamChar(i1, label1);}
    drawCursor(i0, i1, label0, label1);
  }
  i_enc = 0; i_cursor = i1;
}

void updateParam(uint16_t i, int8_t x) 
{
  const uint8_t DATA[3][N_PARAMS] = {
  {1, 3, 50, 1},
  {50, 75, 40, 75},
  {1, 1, 100, 10}};
  const uint16_t p_min = 10*DATA[0][i];           
  const uint16_t p_max = (i < 2) ? DATA[1][i] : DATA[1][i]*100;                                     
  params[i] += x*DATA[2][i];                        // update internally stored float for onscreen param when user turns encoder
  params[i] = constrain(params[i], p_min, p_max);   // constrain param within bounds
}

void getParamChar(uint8_t i, char* paramChar)
{
  const uint8_t p_chars = (i != 2) ? 3 : 4; 
  float param_f = params[i]/10.f;               // convert parameter to floating point
  dtostrf(param_f, p_chars, 1, paramChar);      // create a new string for updated param to be printed next step of loop()
}

void uploadParams()
{
  uint8_t param;                                 // stores the param to be transmitted
  const uint8_t n = 3;                           // number of params
  const uint8_t i_addy[n] = {0, 0, 1};           // arduino address index for iteration
  for (uint8_t i = 0; i < n; i++) 
  {
    Wire.beginTransmission(INO_ADDYS[i_addy[i]]);
    param = (i != 2) ? params[i] : params[i]/100;
    Wire.write(param);
    Wire.endTransmission();
  }
}

void sendInoSignal(uint8_t code)
{
  for(uint8_t i = 0; i < 2; i++)
  {
    Wire.beginTransmission(INO_ADDYS[i]);
    Wire.write(code);
    Wire.endTransmission();
    delay(50);
  }
}

///////////////////// Graphics Helpers /////////////////////
void drawCursor(uint8_t i0, uint8_t i1, char label0[6], char label1[6]) 
{
  Label l0, l1;
  l0.tft = tft; l1.tft = tft;
  l0.text = label0; l1.text = label1;
  const uint16_t x = MARGIN + COL_WIDTHS[0];
  l0.x = x; l1.x = x;
  l0.size = SIZES[LABELS];
  l1.size = SIZES[OPTIONS];
  l0.y = MARGIN + (i0+1)*ROW_HEIGHT;
  l1.y = MARGIN + (i1+1)*ROW_HEIGHT;
  if(blinking) {
    l1.size = SIZES[LABELS];
    l0.color = COLORS[BACK];
    l1.color = (getBlink()) ? COLORS[BACK] : COLORS[TEXT];
    l0.draw();
    l1.draw();
  }
  const bool shrink = (i0 != BTN && !blinking && i0 != i1);
  const bool grow = (i1 != BTN && !blinking);
  if (shrink) {l0.changeSize(-1);}
  if (grow) {l1.changeSize(1);}
  const bool btn_selected = (i1 == BTN);
  btn.select(btn_selected);
  btn.draw();
}

uint16_t getBlink() 
{
  const uint16_t t_blink = 500;                       // time [ms] of cursor blink
  const uint32_t del_t = get_del_t(t_lastBlink);
  const bool cycle = (del_t > 2*t_blink);
  const bool blinked = (del_t > t_blink);             // print text if false, erase if true 
  if (cycle) {t_lastBlink += del_t;}
  return blinked;
}

void drawOptions() 
{
  const char optionName[N_PARAMS][12] = {"Frequency: ", "Stroke: ", "Load: ", "Duration: "};
  const char optionUnit[N_PARAMS][4] =  {"Hz", "Deg", "N", "hr"};
  title.size = SIZES[OPTIONS];
  title.centered = false;
  for(uint8_t i = 0; i < N_PARAMS; i++) 
  {
    title.x = MARGIN; 
    title.y = MARGIN + (i+1)*ROW_HEIGHT;
    for (uint8_t j = 0; j < COLS; j++) 
    {
      char label[12];
      switch(j){
        case 0: memcpy(label, optionName[i], sizeof(optionName[i])); break;
        case 1: getParamChar(i, label); break;
        case 2: memcpy(label, optionUnit[i], sizeof(optionUnit[i])); break;
      }
      title.text = label;
      title.draw();
      title.x += COL_WIDTHS[j];
    }
  }
}

//////////////////////////// Generic Helper Functions ////////////////////////////
unsigned long get_del_t(unsigned long t0) {return millis()-t0;}

uint16_t wrap(int16_t x, const int16_t X0, const int16_t X1) 
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
