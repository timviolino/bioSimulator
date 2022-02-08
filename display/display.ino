///////////////////// Libraries /////////////////////
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>
#include "Button.h"
#include "controlKnob.h"
#include "Label.h"

///////////////////// Pins /////////////////////
#define TFT_RST -1
#define RX 0
#define TX 1
#define KNOB_MSB 2
#define KNOB_LSB 3
#define KNOB_BUTTON 4
#define TFT_DC 9
#define TFT_CS 10

Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);
controlKnob knob = controlKnob(KNOB_MSB, KNOB_LSB, KNOB_BUTTON);


///////////////////// Test Parameters /////////////////////
#define NUM_PARAMS 4
float params[NUM_PARAMS] = {1.0, 3.0, 50.0, 1.0};
const float PARAMS_MAX[NUM_PARAMS] = {5.00, 7.50, 400.0, 744.0};
const float PARAMS_MIN[NUM_PARAMS] = {1.00, 3.00, 50.0, 1};
const float PARAMS_STEP[NUM_PARAMS] = {0.1, 0.1, 10, 1};
const float PARAMS_CHARS[NUM_PARAMS] = {3, 3, 4, 3};

///////////////////// Style /////////////////////
const uint16_t COLORS[3] = {HX8357_BLACK, HX8357_GREEN, HX8357_WHITE};
enum {BACKGROUND, BUTTONS, TEXT};

const uint16_t TEXT_SIZES[3] = {2, 3, 4};
enum {OPTIONS, LABELS, TITLES};
const uint16_t LABEL_SIZES[3] = {TEXT_SIZES[OPTIONS], TEXT_SIZES[OPTIONS], TEXT_SIZES[OPTIONS]};

#define NUM_OPTIONS 5
enum {PARAM, VALUE, UNIT};
enum {FREQUENCY, STROKE, LOAD, HOURS, UPLOAD_BUTTON};
char labels[NUM_PARAMS][3][12] = 
{
  {"Frequency: ", "1.0", "Hz"}, 
  {"Stroke: ", "3.0", "Deg"},
  {"Load: ", "50.0", "N"},
  {"Duration: ", "1.0", "hr"}
};

// Grid dimensions defined based on above
#define WIDTH  tft.width()
#define HEIGHT  tft.height()
#define MARGIN  5
#define ROWS  NUM_OPTIONS + 1
#define COLS  3
#define ROW_HEIGHT  (HEIGHT-2*MARGIN)/(ROWS)
#define COL_WIDTH  (WIDTH-2*MARGIN)/(COLS)

const int COL_WIDTHS[3] = {(WIDTH-2*MARGIN)/2, (WIDTH-2*MARGIN)/3, (WIDTH-2*MARGIN)/6};

uint16_t cursorPos = FREQUENCY;
unsigned long startTime;

///////////////////// Custom Objects /////////////////////
Button *uploadButton;
Button *startButton;
Button *stopButton;
Button *restartButton;
Label *title;


///////////////////// Control Knob /////////////////////
bool encoderMode = true; // selects options if true, selects values if false
int8_t encoderPos;
bool cursorPrinted = false;
const unsigned long blinkTime = 500.0; //ms
unsigned long timeSinceBlink = 0.0; //ms


///////////////////// Machine States /////////////////////
enum {BOOT, USER_INPUT, UPLOAD, RUN_TEST, COMPLETE};
uint8_t state = BOOT;

unsigned long BAUD_RATE = 115200; 


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
      myPrint("Parameters", 0, MARGIN, TEXT_SIZES[TITLES], COLORS[TEXT], true);
      printOptions();
      uploadButton -> draw();
    break;
    
    case UPLOAD:
    myPrint("Parameters Uploaded", 0, MARGIN + 2*ROW_HEIGHT, TEXT_SIZES[OPTIONS], COLORS[TEXT], true);
    myPrint("Mount specimen now!", 0, MARGIN + 3*ROW_HEIGHT, TEXT_SIZES[OPTIONS], COLORS[TEXT], true);
    break;

    case RUN_TEST:
    myPrint("Test in Progress...", 0, MARGIN + 2*ROW_HEIGHT, TEXT_SIZES[OPTIONS], COLORS[TEXT], true);
    printProgressBar();
    break;

    case COMPLETE:
    myPrint("Test Completed", 0, MARGIN + 2*ROW_HEIGHT, TEXT_SIZES[OPTIONS], COLORS[TEXT], true);
    myPrint("Please remove specimen", 0, MARGIN + 3*ROW_HEIGHT, TEXT_SIZES[OPTIONS], COLORS[TEXT], true);
    break;   
  }
}

void initBoot() {
  tft.fillScreen(COLORS[BACKGROUND]);
  title = new Label(tft, "bioSimulator", MARGIN + 2*ROW_HEIGHT, TEXT_SIZES[TITLES]);
  title -> draw(true);
}

///////////////////// Setup Helpers /////////////////////
void mySerialBegin() 
{
  Serial.begin(BAUD_RATE);
  while(!Serial);
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
  encoderPos += knob.readEncoder();
}

void push() 
{
  switch(state) 
  {
    case USER_INPUT:
      if (cursorPos == UPLOAD_BUTTON) 
      {
        nextState();
        break;
      }
      else 
      {
        encoderMode = !encoderMode;
        moveCursor(cursorPos, cursorPos);
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
  if (encoderMode) 
  {
    if (encoderPos != 0) 
    {
      int oldPos = cursorPos;
      cursorPos += encoderPos;
      cursorPos = wrap(cursorPos, FREQUENCY, UPLOAD_BUTTON);
      moveCursor(oldPos, cursorPos);
    }
  }
  else
  {
    if (encoderPos != 0) {
      updateParam(cursorPos, encoderPos);
      moveCursor(cursorPos, cursorPos);
      timeSinceBlink = millis();
    }
    blinkCursor(cursorPos);
  }
  encoderPos = 0;
}

void updateParam(uint16_t i, int8_t x) 
{
  params[i] += x*PARAMS_STEP[i];
  params[i] = constrain(params[i], PARAMS_MIN[i], PARAMS_MAX[i]);
  dtostrf(params[i], PARAMS_CHARS[i], 1, labels[i][1]);
}

uint16_t wrap(int16_t x, int16_t x0, int16_t x1) 
{
  if (x > x1) {return x0;}
  else if (x < x0) {return x1;}
  return x;
}

void upload()
{
  
}
///////////////////// Graphics Helpers /////////////////////
void nextState() 
{
  cursorPos = 0;
  state++;
  tft.fillScreen(COLORS[BACKGROUND]);
  switch(state) 
  {
    case USER_INPUT:
      moveCursor(0, 0);
      uploadButton = new Button(tft, "Upload", MARGIN+(NUM_OPTIONS)*ROW_HEIGHT, HX8357_GREEN);
    break;
    case UPLOAD:
      startButton = new Button(tft, "Start Test", MARGIN+(NUM_OPTIONS)*ROW_HEIGHT, HX8357_GREEN);
      startButton -> select();
      startButton -> draw();
    break;
    case RUN_TEST:
      startTime = millis();
      Serial.println("Stop Test Created");
      stopButton = new Button(tft, "Stop Test", MARGIN+(NUM_OPTIONS)*ROW_HEIGHT, HX8357_RED);
      stopButton -> select();
      stopButton -> draw();
    break;
    case COMPLETE:
      restartButton = new Button(tft, "Restart", MARGIN+(NUM_OPTIONS)*ROW_HEIGHT, HX8357_GREEN);
      restartButton -> select();
      restartButton -> draw();
    break;
  }
}

unsigned long getTimeSince(unsigned long t0) 
{
  const unsigned long t = millis();
  const unsigned long del_t = t-t0;
  return del_t;
}

void printProgressBar()
{
  const int16_t y = MARGIN+(3)*ROW_HEIGHT;
  const uint16_t color0 = HX8357_WHITE;
  const uint16_t color1 = HX8357_GREEN;
  const uint16_t w0 = WIDTH-2*MARGIN; 
  const uint16_t h = ROW_HEIGHT;
  const int16_t x = (WIDTH-w0)/2;
  float del_t = getTimeSince(startTime);
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
    erase("000.0", x, y, TEXT_SIZES[LABELS]);
  }
  else {uploadButton -> deselect();}
  
  // print next cursor
  if (p1 != UPLOAD_BUTTON) 
  {
    label = labels[p1][VALUE];
    y = MARGIN + (p1+1)*ROW_HEIGHT;
    erase(label, x, y, TEXT_SIZES[OPTIONS]);
    myPrint(label, x, y, TEXT_SIZES[LABELS], COLORS[TEXT], false);
  }
  else {uploadButton -> select();}
  
  cursorPrinted = true;
}

void blinkCursor(uint16_t p) 
{
  String label = labels[p][VALUE];
  int x = MARGIN + COL_WIDTHS[0];
  int y = MARGIN + (p+1)*ROW_HEIGHT;
  uint16_t textSize = TEXT_SIZES[LABELS];
  unsigned long del_t = getTimeSince(timeSinceBlink);
   
  if (del_t < blinkTime) {
    myPrint(label, x, y, textSize, COLORS[TEXT], false);
  }
  else if (del_t < 2*blinkTime) {
    erase(label, x, y, textSize);
  }
  else {timeSinceBlink += del_t;}
}

void printOptions() 
{
  int x, y;
  String label;
  
  // print params
  for(uint16_t i = 0; i < NUM_PARAMS; i++) 
  {
    x = MARGIN; y = MARGIN + (i+1)*ROW_HEIGHT;
    for (uint16_t j = 0; j < COLS; j++) 
    {
      label = labels[i][j];
      if (cursorPos != i || j != VALUE){
        myPrint(label, x, y, LABEL_SIZES[j], COLORS[TEXT], false);
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
