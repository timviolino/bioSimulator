////////////////////////////// Libraries //////////////////////////////
#include <SPI.h>                          // Used for packing CAN messages
#include <mcp2515_can.h>                  // Used for packing CAN messages
#include "CubeMarsAK.h"                   // Cube Mars AK Series motor control class 
#include "waveforms.h"                    // array of hardcoded sine wave positions
#include <Wire.h>                         // Used for networking between Arduinos

////////////////////////////// Global Constant Defintions //////////////////////////////
#define CAN_INT_PIN 3                         // attach to arduino pin d3 to connect with can shield int
#define SPI_CS_PIN 9                          // attach to arduino pin d4
#define NUM_MOTORS 3                          // number of motors 
const uint64_t BAUD_RATE = 230400;            // rate of communications over serial bus 
enum {USER_INPUT, BOOT, RUN_TEST, STOP};      // machine states
enum {FREQUENCY, STROKE, LOAD, DURATION};     // indices of parameters
enum {ROLL, PITCH, YAW};                      // indices of motors

mcp2515_can CAN(SPI_CS_PIN);
CubeMarsAK motors[NUM_MOTORS];

////////////////////////////// Test Parameter Variable Declarations //////////////////////////////
// The expected test parameters are as follows:
// frequency in Hz {0.5, 5} 
// stroke in deg {1, 7.5} 
// load in N {10, 400}
// duration in ms

bool powered[3] = {true, true, true};                   // which motors are powered, id needs to be > 2
volatile float params[4] = {5.0f, 2.0f, 10.0f, 0.0f};   // stores parameters for duration of test                                                         
float days = 0.0f, hours = 4.0f, mins = 0.0f;          // easy duration set up 
uint8_t state = BOOT;                                   // current machine state 
float conversionFactor = 0.0f;                          // motor input equivalent to user specified deg's of rotation
uint8_t i_pos = 0;                                      // stores current index used for wave array
uint8_t t_step = 0;                                     // time between steps on wave
unsigned long t_start;                                  // records the time at which the test starts
unsigned long t_lastStep = 0;                           // time at which motors were last actuated

//////////////////////////////// Setup & Loop ////////////////////////////////
void setup() 
{
  mySerialBegin();
  setupMotors();
  //Wire.begin();
}

void setupMotors() 
{
  for (int i = 0; i < NUM_MOTORS; i++) 
  {
    uint8_t id = i+3;
    motors[i].setID(id);
    motors[i].setPower(powered[i]);
  }
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
      ramp(0, params[FREQUENCY]);
      bootTiming();
      state = RUN_TEST;
      break;

    case RUN_TEST:
      oscillate(t_step, 2);
      checkDuration();
      break;

    case STOP:
      ramp(params[FREQUENCY], 0);
      stopTest();
      state = USER_INPUT;
      break;
  }
}

void boot() 
{
  checkCANShield();
  for (int i = 0; i < 3; i++) {motors[i].boot();}
  conversionFactor = degToRad(params[STROKE]);
}

void bootTiming() 
{
  params[DURATION] = getMillis(days, hours, mins);
  t_step = get_t_step(params[FREQUENCY], STEPS/2);
  t_start = millis();
  t_lastStep = t_start;
}

unsigned long get_t_step(float frequency, uint16_t n_steps) {
  float stepsPerSec;
  unsigned long t_step = 1000;
  if(frequency != 0)
  {
    stepsPerSec = float(n_steps * frequency);       // f*120 discrete steps per second
    t_step = int(1000.0f/stepsPerSec);              // ms per step converted to int
  }
  return t_step;
}

void checkDuration() 
{
  unsigned long del_t = get_del_t(t_start);             // record total elapsed time
  if (del_t > params[DURATION]) {state = STOP;}         // if elapsed time is greater than test duration: stop test
}

void oscillate(unsigned long t_step, uint8_t i_step) 
{
  unsigned long del_t = get_del_t(t_lastStep);           // record time since last step
  if (del_t > t_step)                                    // check if time between steps has elapsed
  {               
    t_lastStep += del_t;                                 // update time of last step                            
    float p = getPosition(i_step);                       // record position on sine wave
    for (int i = 0; i < 3; i++) {motors[i].setPos(p);}   // iterate through motors
  }
}

void ramp(float f0, float f1) 
{
  float f = f0;
  const float dir = copysign(1, f1-f0);
  const float f_step = 0.1f;
  const unsigned long t_fstep = 250;
  unsigned long t_last_fstep = millis();
  unsigned long t_step;
  const uint8_t i_step = 4;
  while (dir*(f1-f) > f_step)
  {
    unsigned long del_t = get_del_t(t_last_fstep);
    t_step = get_t_step(f, STEPS/i_step)+1;
    oscillate(t_step, i_step);
    if (del_t > t_fstep) 
    {
      f += dir*f_step;
      t_last_fstep += del_t;
    }
  }
}

void stopTest() 
{
  for (int i = 0; i < 3; i++) 
  {
    motors[i].setPower(false);
    motors[i].boot();
  }
}

float getPosition(uint8_t i_step) 
{
  float p = float(pgm_read_word_near(waveformsTable + i_pos));  // read current value out from table
  p = (p - 2047.5) * conversionFactor / 2047.5;                 // convert to "motor radians"
  i_pos += i_step;                                              // increment position index
  if (i_pos >= STEPS) {i_pos = 0;}                              // reset index at end of array
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

void sendLoad() 
{
  Wire.beginTransmission(9);
  Wire.write(int(params[LOAD]));
  Wire.endTransmission();
}

//////////////////////////// Generic Helper Functions //////////////////////////////////
void mySerialBegin() 
{
  Serial.begin(BAUD_RATE);
  while (!Serial); // wait for serial
}

float getMillis(float days, float hours, float mins) 
{
  return days*86400000.f + hours*3600000.f + mins*60000.f;
}

unsigned long get_del_t(unsigned long t0) 
{
  unsigned long t = millis();
  return t-t0;
}

float degToRad(float deg)
{
  const float K = 0.017125f;      // Note, there is a slight offset per radian in the encoder, hence the deviation from 0.01745 
  return deg*K;
}
