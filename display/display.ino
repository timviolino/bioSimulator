///////////////////// Library Imports /////////////////////
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>
#include "Button.h"
#include "controlKnob.h"
#include "Label.h"

///////////////////// Constant Value Definitions /////////////////////
#define TFT_RST -1
#define RX 0
#define TX 1
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
const uint8_t N_PARAMS = 4;
const uint8_t N_OPTIONS = 5;
const uint8_t S_TEXT[3] = {2, 3, 4};
const uint8_t S_LABEL[3] = {S_TEXT[OPTIONS], S_TEXT[OPTIONS], S_TEXT[OPTIONS]};
const uint8_t PARAMS_CHARS[N_PARAMS] = {3, 3, 4, 3};
const uint8_t PARAMS_MIN[N_PARAMS] = {1, 3, 50, 1};
const float PARAMS_MAX[N_PARAMS] = {5.00, 7.50, 400.0, 744.0};
const float PARAMS_STEP[N_PARAMS] = {0.1, 0.1, 10, 1};
const uint16_t COLORS[3] = {HX8357_BLACK, HX8357_GREEN, HX8357_WHITE};
const uint64_t BAUD_RATE = 115200; 
const uint64_t BLINK_TIME = 500.0;                                    // time [ms] of cursor blink

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
// Grid dimensions defined based on above
#define WIDTH  tft.width()
#define HEIGHT  tft.height()
#define MARGIN  5
#define ROWS  N_OPTIONS + 1
#define COLS  3
#define ROW_HEIGHT  (HEIGHT-2*MARGIN)/(ROWS)
#define COL_WIDTH  (WIDTH-2*MARGIN)/(COLS)
const int COL_WIDTHS[3] = {(WIDTH-2*MARGIN)/2, (WIDTH-2*MARGIN)/3, (WIDTH-2*MARGIN)/6};
controlKnob knob = controlKnob(KNOB_MSB, KNOB_LSB, KNOB_BUTTON);
Button uploadButton = Button("Upload", MARGIN+(N_OPTIONS)*ROW_HEIGHT, HX8357_GREEN);
Button startButton = Button("Start Test", MARGIN+(N_OPTIONS)*ROW_HEIGHT, HX8357_GREEN);
Button stopButton = Button("Stop Test", MARGIN+(N_OPTIONS)*ROW_HEIGHT, HX8357_RED);
Button restartButton = Button("Restart", MARGIN+(N_OPTIONS)*ROW_HEIGHT, HX8357_GREEN);
Label *title;

void setup() 
{
  mySerialBegin();
  initTFT();
  initControlKnob();
}

void loop() 
{
  myReadButton();
  switch(state) 
  {
    case BOOT:
      initBoot();
      delay(1000);
      nextState();
    break;
    
    case USER_INPUT:
      updateCursor();
      myPrint("Parameters", 0, MARGIN, S_TEXT[TITLES], COLORS[TEXT], true);
      printOptions();
      uploadButton.draw();
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

void initBoot() {
  tft.fillScreen(COLORS[BACKGROUND]);
  title = new Label(tft, "bioSimulator", MARGIN + 2*ROW_HEIGHT, S_TEXT[TITLES]);
  title -> draw(true);
}

void initTFT() 
{
  tft.begin();
  tft.setRotation(2);
  tft.setTextWrap(false);
}

void initControlKnob() 
{
   knob.attach();
   attachInterrupt(0, readKnob, CHANGE);
   attachInterrupt(1, readKnob, CHANGE);
}

///////////////////// Control Knob Helpers /////////////////////
void myReadButton()                                      // NOTE this is not an interrupt because both external interrupt pins are used for the encoder
{
  if(knob.getButtonPush()) {push();}
}

void readKnob() 
{
  i_enc += knob.readEncoder();
}

void push() 
{
  switch(state) 
  {
    case USER_INPUT:
      if (i_cursor == UPLOAD_BUTTON) 
      {
        nextState();
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
    break;

    case RUN_TEST:
      state = BOOT;
      nextState();
    break;

    case COMPLETE:
      state = BOOT;
    break;
  }
  Serial.println("state = " + String(state));
}

void updateCursor() 
{
  if (state_enc) 
  {
    if (i_enc != 0) 
    {
      int oldPos = i_cursor;
      i_cursor += i_enc;
      i_cursor = wrap(i_cursor, FREQUENCY, UPLOAD_BUTTON);
      moveCursor(oldPos, i_cursor);
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


void upload()
{
  
}
///////////////////// Graphics Helpers /////////////////////
void nextState() 
{
  i_cursor = 0;
  state++;
  tft.fillScreen(COLORS[BACKGROUND]);
  switch(state) 
  {
    case USER_INPUT:
      moveCursor(0, 0);
      uploadButton.setTFT(tft);
    break;
    case UPLOAD:
      startButton.setTFT(tft);
      startButton.select();
      startButton.draw();
    break;
    case RUN_TEST:
      t_start = millis();
      stopButton.setTFT(tft);
      stopButton.select();
      stopButton.draw();
    break;
    case COMPLETE:
      restartButton.setTFT(tft);
      restartButton.select();
      restartButton.draw();
    break;
  }
}

void printProgressBar()
{
  const int16_t y = MARGIN+(3)*ROW_HEIGHT;
  const uint16_t color0 = HX8357_WHITE;
  const uint16_t color1 = HX8357_GREEN;
  const uint16_t w0 = WIDTH-2*MARGIN; 
  const uint16_t h = ROW_HEIGHT;
  const int16_t x = (WIDTH-w0)/2;
  float del_t = get_del_t(t_start);
  float percent = del_t/(params[HOURS]*10000.f);
  const uint16_t w1 = w0*percent;

  if (w1 < w0) 
  {
    tft.drawRoundRect(x, y, w0, h, 5, color0);
    tft.fillRoundRect(x, y, w1, h, 5, color1);
  }
  else 
  {
    nextState();
  }
}

void moveCursor(uint16_t p0, uint16_t p1) 
{
  String label;
  int x = MARGIN + COL_WIDTHS[0];
  int y;
  // erase last cursor
  if (p0 != UPLOAD_BUTTON)
  {
    label = labels[p0][VALUE]; 
    y = MARGIN + (p0+1)*ROW_HEIGHT;
    erase("000.0", x, y, S_TEXT[LABELS]);
  }
  else {uploadButton.deselect();}
  
  // print next cursor
  if (p1 != UPLOAD_BUTTON) 
  {
    label = labels[p1][VALUE];
    y = MARGIN + (p1+1)*ROW_HEIGHT;
    erase(label, x, y, S_TEXT[OPTIONS]);
    myPrint(label, x, y, S_TEXT[LABELS], COLORS[TEXT], false);
  }
  else {uploadButton.select();}
  
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
unsigned long get_del_t(unsigned long t0) 
{
  const unsigned long t = millis();
  return t-t0;
}

uint16_t wrap(int16_t x, int16_t x0, int16_t x1) 
{
  if (x > x1) {return x0;}
  else if (x < x0) {return x1;}
  return x;
}

void mySerialBegin() 
{
  Serial.begin(BAUD_RATE);
  while(!Serial);
}
