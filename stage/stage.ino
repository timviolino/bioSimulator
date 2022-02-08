#define DEBUG

////////////////////////////// Test Parameter Definitions //////////////////////////////
// The expected test parameters are as follows:
// frequency in Hz {0.5, 5} 
// stroke in deg {1, 7.5} 
// load in N {10, 400}
// duration in ms

enum motorEnum {ROLL, PITCH, YAW};                      // CAN id - 1 of motors
bool powered[3] = {true, false, false};                 // which motors are powered needs to be > 2
enum paramEnum {FREQUENCY, STROKE, LOAD, DURATION};     // indices of parameters
volatile float params[4] = {2.0f, 7.5f, 10.0f, 0.0f};   // stores parameters for duration of test                                                         
float days = 0.0f, hours = 0.0f, mins = 1.00f;          // easy duration set up 

////////////////////////////// Machine States //////////////////////////////
enum states {USER_INPUT, BOOT, RUN_TEST, STOP};
uint8_t state = BOOT;

////////////////////////////// Pin Definitions //////////////////////////////
#define CAN_INT_PIN 3
#define SPI_CS_PIN 9

////////////////////////////// Library Imports //////////////////////////////
#include <SPI.h>                          // Used for packing CAN messages
#include <mcp2515_can.h>                  // Used for packing CAN messages
mcp2515_can CAN(SPI_CS_PIN);

#include <Wire.h>                         // Used for networking between Arduinos
#include "motorMode.h"                    // Custom, used to organize motor communications
#include "motorPackets.h"                 // Custom, used for organize motor communications
#include "waveforms.h"                    // array of hardcoded sine wave positions
#include <math.h>

const unsigned long BAUD_RATE = 115200;

////////////////////////////// time/position definitions //////////////////////////////
float cmds[5] = {0.0f, 0.0f, 30.0f, 0.5f, 0.0f};    // {P, V, KP, KD, To};
float conversionFactor = 0.0f;                      // motor input equivalent to user specified deg's of rotation
volatile float zeroFactors[3] = {0.0f, 0.0f, 0.0f}; // used to adjust the zero point of commands sent to motor

////////////////////////////// time/position definitions //////////////////////////////
uint8_t timeStepGlobal = 0;               // time between steps on wave
uint8_t posIndex = 0;                     // stores current index used for wave array
unsigned long times[2] = {0.0, 0.0};      // used for measuring whether a cycle has elapsed
unsigned long timeStart;                  // records the time at which the test starts


//////////////////////////////// Setup & Loop ////////////////////////////////
void setup() 
{
  mySerialBegin();
  //Wire.begin();
}

void loop() 
{
  switch (state) 
  {
    case USER_INPUT:
      //readUserInput();
      break;

    case BOOT:
      boot();
      //ramp(0, params[FREQUENCY]);
      state = RUN_TEST;
      break;

    case RUN_TEST:
      oscillate(timeStepGlobal);
      checkDuration();
      break;

    case STOP:
      //ramp(params[FREQUENCY], 0);
      stopTest();
      state = USER_INPUT;
      break;
  }
}

void boot() 
{
  checkCANShield();
  initMotors();
  params[DURATION] = getMillis(days, hours, mins);
  timeStepGlobal = getStepTime(params[FREQUENCY]);
  conversionFactor = 2 * params[STROKE] * 0.017125f;
  timeStart = millis();
  times[0] = timeStart;
}

uint8_t getStepTime(float frequency) {
  float stepsPerSec, msPerStep;
  int delta_t;
  if(frequency != 0) 
  {
    stepsPerSec = float(STEPS * frequency);
    msPerStep = 1000.0f/stepsPerSec;
    delta_t = int(msPerStep);
  }
  else
  {
    delta_t = 10000;
  }
  return delta_t;
}

void checkDuration() 
{
  unsigned long elapsedTime = times[1] - timeStart;   // record total elapsed time
  if (elapsedTime > params[DURATION]) 
  {
    state = STOP;                                     // if elapsed time is greater than test duration: stop test
  }
}

void oscillate(uint8_t stepTime) 
{
  times[1] = millis();                                // check what time it is
  if (times[1] - times[0] > stepTime)                 // else if time between steps has elapsed, actuate motor
  {               
    times[0] = times[1];                              // update time value
    float p = getPosition();                          // record current step of path
    for (int i = 0; i < 3; i++)                       // iterate through motors
    {                     
      if (powered[i])                                 // check if motor is on
      {                               
        cmds[P] = p + zeroFactors[i];                 // update position for motor #i, include zero factor
        pack_cmd(i + 5, cmds);                        // send packet via CAN-Bus
        unpack_reply();                               // receive packet via CAN-Bus
        #ifdef DEBUG
          Serial.println("");
        #endif
      }
    }
  }
}

void ramp(float f0, float f1) 
{
  float delta_f = f1-f0;
  float dir = copysign(1, delta_f);
  uint8_t i0, i1, delta_t;
  while (dir*(f1-f0) >= 0)
  {
    delta_t = getStepTime(f0);
    i0 = posIndex;
    oscillate(delta_t);
    i1 = posIndex;
    if (i0 != i1 && i0 % 30 == 0) 
    {
      f0 += dir*0.2;
    }
  }
}

void stopTest() 
{
  for (int i = 0; i < 3; i++) 
  {
    powered[i] = false;
  }
  initMotors();
}

float getPosition() 
{
  float p = 0.0f;
  p = float(pgm_read_word_near(waveformsTable + posIndex));  // read current value out from table
  p = (p - 2047.5) * conversionFactor / 2047.5;              // convert to "motor radians"
  posIndex++;                                                // increment position index
  if (posIndex == STEPS) 
  {
    posIndex = 0;                                            // reset index at end of array
  }
  return p;
}

void checkCANShield() 
{
  while (CAN_OK != CAN.begin(CAN_1000KBPS))              // init can bus : baudrate = 1MHz
  {
#ifdef DEBUG
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
#endif
    delay(100);
  }
#ifdef DEBUG
  Serial.println("CAN BUS Shield init ok!");
#endif
}

void initMotors() 
{
  for (int i = 0; i < 3; i++)  
  {
    if (powered[i] == 1) 
    {
      int id = i+5;
      enterMotorMode(id);
      delay(2500);
      zeroFactors[i] = readPosition();
      #ifdef DEBUG
        Serial.println("ID: " + String(id) + " Zero Factor = " + String(zeroFactors[i]));
      #endif
    }
    else 
    {
      exitMotorMode(i);
    }
  }
}

void mySerialBegin() 
{
  Serial.begin(BAUD_RATE);
  while (!Serial); // wait for serial
}

float getMillis(float days, float hours, float mins) 
{
  return days*86400000.f + hours*3600000.f + mins*60000.f;
}

void sendLoad() 
{
  Wire.beginTransmission(9);
  Wire.write(int(params[LOAD]));
  Wire.endTransmission();
}
